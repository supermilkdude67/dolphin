I've only tested it so far on Linux, but it should build fine on Windows.

Feedback is welcome, as its very buggy currently.

At the moment on my setup, I've tested it with LEGO Rock Band and it was detected by the game, but then no audio input would come up in game.
(I also tested by replacing audio input with a sine wave, and I can hear that playback in the game, so the stream buffer is probably empty)

Its mostly based off of the Wii Speak emulation code, so at the moment things like the configuration window are nearly identical to the Wii Speak's
configuration window. (Rock Band 3 requires 3 microphones to be plugged in to play harmonies, and Guitar Hero 5, Band Hero, and Warriors of Rock allow 4 microphones in multiplayer, so in the future I'm gonna add support for 4 simultaneous microphones)

Link to original Dolphin repo: https://github.com/dolphin-emu/dolphin/
