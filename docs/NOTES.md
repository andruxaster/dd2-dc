# Findings, quirks and open checks

What was learnt about the PSX game while porting it, where the port deliberately differs, and what has not
been checked against the real game yet. Differences in the code are marked `PORT:`; list new ones here.

## Architecture

- `src/psx/` replaces Sony's Psy-Q SDK: `libgte` software GTE, `libgpu` VRAM 1024×512 with OT and a primitive
  rasterizer, `libspu` voices/ADPCM, `libcd` CdSearchFile/CdRead on `assets/` plus CD-DA, `libpad` pad from SDL
  gamepad/keys, `libcard` memory card files, `libetc` VSync/callbacks, `libapi` events/heap, `libpress` MDEC.
- Overlays (FRONTEND, GENERAL, RESULTS) are normal source files linked into the exe; overlay loads are no-ops.
  Loads to fixed addresses (`DRIVERS.BIN` → `0x8018C000`, `InitHeap(.., 0xE0000)`, scratchpad `0x1F800000`)
  go to buffers.
- The intro and outro movies are separate executables on the PSX (NTSCINTR.EXE, NTSCXTRO.EXE and the PAL ones,
  Sony's streaming movie sample); their code is `game/movie.c`, run in-process by `Play_Intro`/`Play_Xtro`.

## Timing

- Races run one game frame per two vblanks (`Draw_All(2)`: 29.97 a second on NTSC), measured over 600 demo
  frames. A `VSync(-1)` that slept a vblank on every repeated read once made `Draw_All(2)` take 3 vblanks
  (races at two-thirds speed); `VSync(-1)` must only read the counter.
- `Play_Game` does exactly one game step per drawn frame, never a number derived from the measured frame time.
- Replay slow motion (hold Cross) waits two vblanks before the frame's work; on the PSX that work takes over a
  vblank, so a frame takes 3 vblanks (two-thirds speed). The port's work takes no time, so it waits a third
  vblank in its place (`PORT:` in `play_game.c`). Assumes a replay frame needs one to two vblanks of CPU, as a
  20-car race does.
- Movies stream at double speed: 150 sectors a second, on the audio clock.

## Sound

- SPU pitch `0x1000` = 44.1 kHz (sample rate × pitch / 4096); the voices use the hardware's Gaussian
  interpolation, ADSR and loop flags (`tests/spu_test.c`).
- `Update_Race_Sounds` gives engines to the player and the 7 nearest cars: pitch by speed and gear, Doppler,
  distance, pan.
- The game sets CdMix to 255 (about 2×): CD music clips like on hardware (≈1.5% of samples on the menu track),
  and so does the movies' XA sound.
- Movie XA sound goes to 44.1 kHz by linear interpolation, not the hardware's 7-phase filter.
- The game has not been seen turning on volume sweeps, reverb, noise or pitch modulation, so `libspu` does not
  emulate them; add them if it does.
  `Sound_Reverb_On`/`Off`, `Sound_Set_Reverb` and `Sound_Set_Reverb_Depth` (0x8004E7A0..) have no callers and
  were not ported.
- `Play_Sound` returns 0/1 instead of a channel when the sample rate is too low for one frame (original bug,
  kept).
- `Allocate_Sound_Effect` drops an effect when no channel is free; the original writes before the table
  (`PORT:`).

## Original quirks kept

- Car-to-car collisions turn the second car's roll instead of its heading; the head-on side table is always
  indexed 0; `Knock_Car_Pitch` takes the edge from the first table for both directions.
- The flying-object ring is stepped with `& 0x17`, so only records 16..23 are used (eight parts at once).
- Dents of the computer cars go through the near model's shared mesh, whose vertices point at the car drawn
  last (`Dent_Car_Zone`, `Boot_Lost_Geometry` for car 0), as the PSX code reads.
- The configuration save puts `champ_info` 4 bytes into the last season's statistics: a load gives those bytes
  back as the first name's letters.
- The statistics pages number the seasons from `seasons_played + 1` once more than five were played (the oldest
  kept is `seasons_played - 3`).
- `rand` is the BIOS generator and the game never seeds it.
- The distant car model (`SECTION_CAR_FAR`, drawn beyond z 5000) has a flat rear: its bumper quad maps a
  2-texel strip (page 0x0d, v 192..193) through the car's `SMCL..B` CLUT, entries 7 and 8. For several paints
  entry 8 is light grey, so far yellow or red cars show a grey rectangle at the back until the near model
  takes over. Checked against `Apply_Car_Part_Cluts` (0x80034AB4), its table at 0x800708F8 and the 0x1388
  switch in `Draw_Car`; the software reference and the PVR path draw it the same.

## Port deviations (`PORT:`)

- Dreamcast (`DD2_FAST_GTE`, on by default in `Makefile.dc`): RTPS/RTPT project in single-precision floats
  (`gte.c`). Against the exact arithmetic on 200,000 random vertices: SX or SY one off in about 2%, SZ in
  0.06%, never more than one; the flags differed 7 times (values on a clamp edge). The desktop build stays
  exact.
- Dreamcast: the CD sound (music, XA) goes through a 371 ms KOS stream ring, so it plays 190-370 ms after
  the game makes it; with a smaller ring a slow frame ran it dry and the music stuttered
  (`platform_kos.c`). Sound effects play on AICA voices with no such delay.
- `Print_Bank_Fade` with transparency mode 2 reads uninitialised registers in the original; the port does not
  fade.
- `Place_Cars_On_Grid` seeds the fd position.
- The bowl AI refuses a target pick of 20 (one past the cars) and picks the player when its car list is used up.
- The load/save screen reads the block before a card's table when the chosen card is unformatted or gone
  (block -1); the port takes it as an empty block (`loadsave.c`).
- `OpenEvent` returns the open event for the same class and kind (`libapi.c`): the game opens the card events
  again after every movie.
- Movie decoding is not bit-exact: the IDCT's intermediate rounding is ours (VLC, quantisation and IDCT tables
  are the PSX ones).

## Port decisions

- Dreamcast (`platform_kos.c`): the display area becomes a 1024x512 PVR texture drawn over 640x480; a frame
  is skipped when the PVR is still busy, never delayed. Sound goes through a ring into a KOS sound stream.
  Memory cards stay in memory (`platform_card_path` returns false; 128 KB does not fit on a VMU). Mapping in
  `README.md`.
- Objects project each vertex once per draw (`draw_face.c`: a table by vertex index, made when a face first
  needs the vertex; `gte_project` and `gte_rtpt_projected` replay RTPT exactly). A new object and the sprite
  runs (which change the rotation and translation) start the table afresh. Checked in a practice race and
  the demo: 5 million RTPT/RTPS replays, every one leaving the GTE as the plain instruction did.
- RTPS/RTPT keep the last results per vertex (`gte.c`, 128 entries) for the current rotation, translation,
  offsets, projection distance and depth cue: faces share vertices, and about 60% of projections in a race
  repeat one. Exact (the context is compared at every RTPS/RTPT; fuzzed against the plain version with
  random context changes). On the SH simulator a repeated vertex costs about 190 instructions, a new one
  about 490, against 343 before.
- `Makefile.dc` defines `DD2_SKIP_UNCHANGED_FRAMES` (libgpu does not show a picture again when nothing
  changed VRAM or the display since: the game draws at 30 Hz, the vblank is 60 Hz) and by default
  `DD2_PROFILE` (`psx/profile.h`: game fps and the share of time in the GPU, SPU and present, on screen
  and on the console).
- Dreamcast PVR renderer (`DD2_HW_RENDER`, `psx/gpu_hw.h`, `platform/kos_gpu.c`): primitives drawn into a
  screen buffer become PVR polygons recorded per buffer and shown when the game displays it; draw order
  becomes depth. Texture pages are converted through their CLUT into a cache, invalidated by VRAM writes.
  What the CPU puts in a screen buffer is drawn under the primitives. Drawing outside the screens stays in
  software. Semi-transparent textured primitives are drawn twice (STP-clear texels solid, STP-set ones
  blended), as the PSX decides per texel. `PORT:` approximations are listed at the top of `kos_gpu.c` (mode 2 as B*(1-F), modulation capped at 2x, no dither/mask/clip, screen read-backs see no primitives).
  The pause menu's copy of the frozen picture is followed at the record level.
- Dreamcast AICA voices (`DD2_SPU_HW`, `psx/spu_hw.h`, `platform/kos_spu.c`): libspu still runs every voice's
  state exactly (envelope, position, loop flags, ENDX; `voice_advance` jumps between envelope steps and
  block ends, checked equal to `voice_sample` stepping on 240,000 random stretches) but makes no sound;
  each voice plays on an AICA channel from its sample decoded once to 16-bit PCM (samples over 65535 are
  averaged down 2x or 4x and played slower), pitch, volume and pan updated per vblank. Only the CD audio
  goes through the stream. `PORT:` notes in libspu.c (vblank-rate volume, hardware interpolation, loop
  history).
- `gpu.c` triangles step the edge functions and attribute numerators per pixel and divide by the area with an
  exact reciprocal multiply: same pixels as the per-pixel divisions (checked on 200,000 random polygons and
  every area at the boundary numerators), but no division per pixel, which the SH4 does in software.

- Memory cards are 128 KB raw card images (the format emulators use) in the user's data folder
  (`SDL_GetPrefPath`): `memcard1.mcd` is made, formatted, when missing; slot 2 is only there when
  `memcard2.mcd` exists. Test runs use a card in memory only.
- Save files keep the PSX layout byte for byte, so real card images load. Replay saves store the end of the
  script as a PSX address (0x8009581C + offset); the port converts it and keeps a damaged one inside the script.
- `--negcon` makes pad 1 a NeGcon (twist from the left stick, I/II from the triggers, L from the left shoulder);
  the default is a digital pad. Test runs skip the intro unless `--intro` is given.

## Code and data findings

- Many PSX globals are initialised data, not BSS (`0x80010000..0x80079FFF` in the exe): read the exe image
  (`tools/exe_data.py`) before declaring one without an initialiser. Zeroing them broke the camera mode (no
  grid fly-in), the car shadow sprite (subtractive grey 0x40; zeroed it drew an opaque dark quad), the particle
  and wheel sprites and the 2:30.00 track records.
- Not ported because the PSX never runs them: `Show_Analog_Screen` and `Show_NeGcon_Screen` (FRONTEND
  0x801CC1E0, 0x801CE848; `Select_Control_Method` turns a NeGcon/analog type into "no pad" first),
  `Labelled_Button_Pressed` (0x801D09A0, no callers). Ghidra splits two pieces off `Centre_Screen_Page`
  (FRONTEND 0x801C72D0, 0x801C7304); they are not functions.
- `Decompress`, `Set_Glyph_Quad_Attrs` and `Draw_Font_Poly` sit with the draw-buffer code in the exe and are in
  `object.c`; that block may be an original module of its own.
- Names: struct gaps no PSX code touches are `unused_XX` (checked by displacement scans in Ghidra and the level
  files), vector fourth words `pad` or `[3]`. What is left in hex is tuning, geometry and table data.

## Checking changes

- Refactors are checked by running the same build twice: a demo race, a scripted player race and the menu give
  byte-identical screenshots and `--wav` recordings before and after.

## Open checks against the real game

- Time a lap in DuckStation to confirm the race speed.
- Compare a recording (`dd2 --wav out.wav --after N`) with an emulator recording of the same moment; engine
  sounds have only been checked with a channel trace, not by ear.
- Compare the music and XA clipping (CdMix 255) by ear.
- Whether a hit computer car or another car shows the dent (shared mesh, above).
- Debris and lost parts were seen in a run (a bonnet flies off) but not compared.
- The grey rear of distant cars (above) in DuckStation, to see it with the original GPU.
- The boot seed of `rand` in `psx/libc.c` (0x24040001) is unverified.
- Front-end screens written but only partly tried by hand: race mode/type, car and track select, CD player,
  controller and sound config, centre screen, info menu, fastest laps, credits, name entry and cheat names.

## Ideas not started

- Widescreen, a hi-res renderer behind `libgpu`, options.
