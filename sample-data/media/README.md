# Demo media

Place media files here (they are intentionally not committed):

- `museum-tour.mp4` for camera + optional embedded audio;
- `robot-audio.wav` for a dedicated robot microphone source.

Set paths as container paths, for example:

```env
SIMULATOR_MEDIA_SOURCE_TYPE=file
SIMULATOR_MEDIA_SOURCE=/media/museum-tour.mp4
SIMULATOR_AUDIO_SOURCE=/media/robot-audio.wav
```

