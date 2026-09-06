/*
 * Copyright (C) 2026 Mark Hills <mark@xwax.org>
 *
 * This file is part of "xwax".
 *
 * "xwax" is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 as
 * published by the Free Software Foundation.
 *
 * "xwax" is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses/>.
 *
 */

#define _GNU_SOURCE
#include <assert.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <SDL.h>

#include "device.h"
#include "sdl.h"

#define DEFAULT_RATE 48000
#define DEFAULT_BUFFER 512
#define MAX_BLOCK 512

#define FIFO_FRAMES 8192
#define FIFO_MASK (FIFO_FRAMES - 1)
#define MAX_LAG_FRAMES 2048

static_assert((FIFO_FRAMES & (FIFO_FRAMES - 1)) == 0, "FIFO_FRAMES must be a power of 2");

struct sdl_fifo {
    signed short *buf;
    atomic_size_t head;
    atomic_size_t tail;
};

struct sdl {
    SDL_AudioDeviceID playback_dev;
    SDL_AudioDeviceID capture_dev;
    unsigned int rate;
    bool started;
    struct sdl_fifo fifo;
};

static unsigned int sdl_deck_count = 0;

static int sdl_fifo_init(struct sdl_fifo *f)
{
    f->buf = malloc(FIFO_FRAMES * DEVICE_CHANNELS * sizeof(signed short));
    if (f->buf == NULL) {
        perror("malloc");
        return -1;
    }

    atomic_init(&f->head, 0);
    atomic_init(&f->tail, 0);
    return 0;
}

static void sdl_fifo_clear(struct sdl_fifo *f)
{
    free(f->buf);
    f->buf = NULL;
}

static void capture_callback(void *userdata, Uint8 *stream, int len)
{
    struct device *dv = (struct device*)userdata;
    struct sdl *sdl = (struct sdl*)dv->local;
    size_t frames = len / (DEVICE_CHANNELS * sizeof(signed short));
    signed short *pcm = (signed short*)stream;

    if (!sdl->started)
        return;

    size_t head = atomic_load_explicit(&sdl->fifo.head, memory_order_relaxed);
    size_t tail = atomic_load_explicit(&sdl->fifo.tail, memory_order_acquire);
    size_t occupied = head - tail;
    size_t space = (occupied < FIFO_FRAMES) ? (FIFO_FRAMES - occupied) : 0;
    size_t to_write = (frames < space) ? frames : space;

    for (size_t i = 0; i < to_write; i++) {
        size_t idx = (head + i) & FIFO_MASK;
        sdl->fifo.buf[idx * DEVICE_CHANNELS]     = pcm[i * DEVICE_CHANNELS];
        sdl->fifo.buf[idx * DEVICE_CHANNELS + 1] = pcm[i * DEVICE_CHANNELS + 1];
    }

    atomic_store_explicit(&sdl->fifo.head, head + to_write, memory_order_release);
}

static void playback_callback(void *userdata, Uint8 *stream, int len)
{
    struct device *dv = (struct device*)userdata;
    struct sdl *sdl = (struct sdl*)dv->local;
    size_t playback_frames = len / (DEVICE_CHANNELS * sizeof(signed short));
    signed short *pcm_out = (signed short*)stream;

    if (!sdl->started) {
        memset(stream, 0, len);
        return;
    }

    /* Drain captured timecode from the FIFO into timecoder */

    if (sdl->capture_dev > 0) {
        size_t head = atomic_load_explicit(&sdl->fifo.head, memory_order_acquire);
        size_t tail = atomic_load_explicit(&sdl->fifo.tail, memory_order_relaxed);
        size_t available = head - tail;

        /* If capture audio lagged excessively (e.g. playback stall), drop older frames */
        if (available > MAX_LAG_FRAMES) {
            tail = head - MAX_LAG_FRAMES;
            available = MAX_LAG_FRAMES;
        }

        while (available > 0) {
            signed short buf[MAX_BLOCK * DEVICE_CHANNELS];
            size_t block = (available < MAX_BLOCK) ? available : MAX_BLOCK;

            for (size_t i = 0; i < block; i++) {
                size_t idx = (tail + i) & FIFO_MASK;
                buf[i * DEVICE_CHANNELS]     = sdl->fifo.buf[idx * DEVICE_CHANNELS];
                buf[i * DEVICE_CHANNELS + 1] = sdl->fifo.buf[idx * DEVICE_CHANNELS + 1];
            }

            device_submit(dv, buf, block);

            tail += block;
            available -= block;
        }

        atomic_store_explicit(&sdl->fifo.tail, tail, memory_order_release);
    }

    /* Collect audio from player for playback in blocks of MAX_BLOCK */

    size_t remain = playback_frames;
    while (remain > 0) {
        size_t block = (remain < MAX_BLOCK) ? remain : MAX_BLOCK;

        device_collect(dv, pcm_out, block);
        pcm_out += block * DEVICE_CHANNELS;
        remain -= block;
    }
}

static unsigned int sample_rate(struct device *dv)
{
    struct sdl *sdl = (struct sdl*)dv->local;
    return sdl->rate;
}

static void start(struct device *dv)
{
    struct sdl *sdl = (struct sdl*)dv->local;

    assert(dv->timecoder != NULL);
    assert(dv->player != NULL);

    sdl->started = true;

    if (sdl->capture_dev > 0)
        SDL_PauseAudioDevice(sdl->capture_dev, 0);

    if (sdl->playback_dev > 0)
        SDL_PauseAudioDevice(sdl->playback_dev, 0);
}

static void stop(struct device *dv)
{
    struct sdl *sdl = (struct sdl*)dv->local;

    sdl->started = false;

    if (sdl->playback_dev > 0)
        SDL_PauseAudioDevice(sdl->playback_dev, 1);

    if (sdl->capture_dev > 0)
        SDL_PauseAudioDevice(sdl->capture_dev, 1);
}

static void clear(struct device *dv)
{
    struct sdl *sdl = (struct sdl*)dv->local;

    if (sdl->playback_dev > 0)
        SDL_CloseAudioDevice(sdl->playback_dev);

    if (sdl->capture_dev > 0)
        SDL_CloseAudioDevice(sdl->capture_dev);

    sdl_fifo_clear(&sdl->fifo);
    free(sdl);

    assert(sdl_deck_count > 0);
    sdl_deck_count--;

    if (sdl_deck_count == 0)
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

static struct device_ops sdl_ops = {
    .sample_rate = sample_rate,
    .start = start,
    .stop = stop,
    .clear = clear,
};

static void print_audio_devices(void)
{
    int num_play = SDL_GetNumAudioDevices(0);
    int num_cap = SDL_GetNumAudioDevices(1);

    fprintf(stderr, "\nAvailable SDL Audio Playback Devices (%d):\n", num_play);
    for (int i = 0; i < num_play; i++)
        fprintf(stderr, "  [%d] %s\n", i, SDL_GetAudioDeviceName(i, 0));

    fprintf(stderr, "\nAvailable SDL Audio Capture Devices (%d):\n", num_cap);
    for (int i = 0; i < num_cap; i++)
        fprintf(stderr, "  [%d] %s\n", i, SDL_GetAudioDeviceName(i, 1));

    fprintf(stderr, "\n");
}

static const char* resolve_device_name(const char *spec, int iscapture)
{
    if (spec == NULL || spec[0] == '\0' || !strcasecmp(spec, "default"))
        return NULL;

    if (!strcasecmp(spec, "none") || !strcmp(spec, "-"))
        return (const char*)-1;

    char *endptr;
    long idx = strtol(spec, &endptr, 10);
    int num = SDL_GetNumAudioDevices(iscapture);

    if (*endptr == '\0' && idx >= 0 && idx < num)
        return SDL_GetAudioDeviceName((int)idx, iscapture);

    /* Search for exact match */
    for (int i = 0; i < num; i++) {
        const char *name = SDL_GetAudioDeviceName(i, iscapture);
        if (name != NULL && !strcmp(spec, name))
            return name;
    }

    /* Search for case-insensitive substring match */
    for (int i = 0; i < num; i++) {
        const char *name = SDL_GetAudioDeviceName(i, iscapture);
        if (name != NULL && strcasestr(name, spec) != NULL)
            return name;
    }

    return spec;
}

int sdl_init(struct device *dv, const char *name, unsigned int rate,
             unsigned int buffer)
{
    struct sdl *sdl;
    SDL_AudioSpec desired, obtained_play, obtained_cap;
    const char *play_spec, *cap_spec;
    const char *play_dev_name, *cap_dev_name;
    char *name_copy = NULL;

    if (!SDL_WasInit(SDL_INIT_AUDIO)) {
        if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
            fprintf(stderr, "SDL: Failed to initialise audio: %s\n", SDL_GetError());
            return -1;
        }
    }

    if (name != NULL && (!strcasecmp(name, "help") ||
                         !strcasecmp(name, "list") ||
                         !strcmp(name, "?")))
    {
        print_audio_devices();
        return -1;
    }

    if (name != NULL && strchr(name, ',') != NULL) {
        char *comma;

        name_copy = strdup(name);
        if (name_copy == NULL) {
            perror("strdup");
            return -1;
        }

        comma = strchr(name_copy, ',');
        *comma = '\0';
        play_spec = name_copy;
        cap_spec = comma + 1;
    } else {
        play_spec = name;
        cap_spec = name;
    }

    play_dev_name = resolve_device_name(play_spec, 0);
    cap_dev_name = resolve_device_name(cap_spec, 1);

    sdl = malloc(sizeof *sdl);
    if (sdl == NULL) {
        perror("malloc");
        free(name_copy);
        return -1;
    }

    memset(sdl, 0, sizeof *sdl);

    if (sdl_fifo_init(&sdl->fifo) == -1) {
        free(sdl);
        free(name_copy);
        return -1;
    }

    SDL_zero(desired);
    desired.freq = rate ? rate : DEFAULT_RATE;
    desired.format = AUDIO_S16SYS;
    desired.channels = DEVICE_CHANNELS;
    desired.samples = buffer ? buffer : DEFAULT_BUFFER;
    desired.callback = playback_callback;
    desired.userdata = dv;

    sdl->playback_dev = SDL_OpenAudioDevice(play_dev_name, 0, &desired, &obtained_play, 0);
    if (sdl->playback_dev == 0) {
        fprintf(stderr, "SDL: Failed to open playback device '%s': %s\n",
                play_dev_name ? play_dev_name : "default", SDL_GetError());
        sdl_fifo_clear(&sdl->fifo);
        free(sdl);
        free(name_copy);
        return -1;
    }

    sdl->rate = obtained_play.freq;

    if (cap_dev_name != (const char*)-1) {
        desired.callback = capture_callback;
        desired.userdata = dv;

        sdl->capture_dev = SDL_OpenAudioDevice(cap_dev_name, 1, &desired, &obtained_cap, 0);
        if (sdl->capture_dev == 0) {
            fprintf(stderr, "SDL: Warning: capture device '%s' not available (%s); running playback only\n",
                    cap_dev_name ? cap_dev_name : "default", SDL_GetError());
        }
    }

    fprintf(stderr, "SDL: Initialised deck on '%s' (%uHz, %u samples)",
            play_dev_name ? play_dev_name : "default", sdl->rate, obtained_play.samples);
    if (sdl->capture_dev > 0)
        fprintf(stderr, ", capture from '%s'\n", cap_dev_name ? cap_dev_name : "default");
    else
        fprintf(stderr, ", capture disabled\n");

    free(name_copy);

    device_init(dv, &sdl_ops);
    dv->local = sdl;
    sdl_deck_count++;

    return 0;
}
