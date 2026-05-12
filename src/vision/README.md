# vision

Maskinsyn-pakke for UR-prosjektet i AIS2105. Pakka eier alt fra raa kamera-input
til fargedeteksjon av kuber. Roboten styres av kollegaens MoveIt-stack i de
oeverige pakkene (`bringup`, `control`, `description`, `moveit_config`) — denne
pakka leverer kun deteksjon-output som planner-noden konsumerer.

## Fasing

- **Fase 1 (paagaaende):** Frittstaaende kamera-pipeline paa laptop, ingen
  robot-tilkobling noedvendig. Maal: kamera-stroem inn, HSV-deteksjon ut,
  lagring av deteksjoner til CSV for rapportbruk.
- **Fase 2:** Integrasjon med robot. Kamera flyttes til TCP-montering,
  URDF utvides med `camera_link`, og deteksjoner publiseres som
  `geometry_msgs/PoseStamped` i `ur3_base_link`-frame via TF.

## Noder

| Node | Status | Beskrivelse |
|---|---|---|
| `usb_cam` (ekstern) | Klar | Driver fra `ros-jazzy-usb-cam`. Publiserer `/image_raw` + `/camera_info`. |
| `hsv_tuner` | Planlagt | OpenCV-trackbars for live tuning av HSV-grenser per farge. |
| `hsv_detector` | Planlagt | Subscriber `/image_raw`, publiserer `/vision/detections`. |
| `detection_logger` | Planlagt | Logger deteksjoner til CSV med tidsstempel + lysforhold. |

## Kjoring

Bygg pakka isolert (uten kollegaens UR-pakker):

```bash
cd ~/ros2_ws/UR-prosjekt
colcon build --packages-select vision
source install/setup.bash
```

Start kamera:

```bash
ros2 launch vision camera.launch.py
```

Verifiser bildestroem (ny terminal):

```bash
ros2 topic hz /image_raw                # forventet 30 Hz
ros2 run rqt_image_view rqt_image_view  # velg /image_raw fra dropdown
```

## Maskinvare

- **Kamera:** Logitech HD Pro Webcam C920 (USB-ID `046d:082d`)
- **Device node:** `/dev/video2` (det integrerte laptop-kameraet er `/dev/video0`)
- **Format:** MJPG @ 1280x720 @ 30 fps (se `config/camera_params.yaml` for begrunnelse)

## Designvalg og metodevurdering

Kort begrunnelse av valg som vil utvides i rapporten:

- **HSV-thresholding fremfor feature-baserte metoder.** Feature-baserte
  bibliotek som `find_object_2d` (ORB/SIFT/SURF) krever teksturerte
  objekter. Solide ensfargede kuber har for faa feature-punkter til at
  slike metoder fungerer paalitelig. HSV-mask + konturanalyse er
  standard fremgangsmaate for fargebaserte objekter (Bradski & Kaehler,
  *Learning OpenCV 3*, kap. 9).
- **MJPG som pixel-format.** Paakrevd for 30 fps over USB 2.0 ved 720p
  og hoeyere oppløsning paa C920. Verifisert med `v4l2-ctl
  --list-formats-ext`.
- **Bygging av paragraf fra Lab4-malen.** Pakka gjenbruker
  node-mønsteret fra `camera_pipeline` i Lab4: cv_bridge-konvertering,
  topic-remapping i launch, og ament_python-struktur.

## Kilder

Akkumuleres etterhvert som koden vokser. Brukes som referanseliste i
sluttrapporten.

| Tema | Referanse |
|---|---|
| ROS2 kamera-driver | usb_cam, https://github.com/ros-drivers/usb_cam |
| Image-konvertering | cv_bridge, https://docs.ros.org/en/jazzy/p/cv_bridge/ |
| OpenCV HSV-fargerom | OpenCV docs, https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html |
| Klassisk maskinsyn | Bradski & Kaehler, *Learning OpenCV 3*, O'Reilly (2017) |
| Generelt | Szeliski, *Computer Vision: Algorithms and Applications*, 2. utg. (2022) |
| Vurdert og forkastet | Labbe, find_object_2d, http://wiki.ros.org/find_object_2d (krever teksturerte objekter) |
| Tidligere arbeid | Lab4 camera_pipeline (AIS2105, varsemester 2026) |

## Opphav per fil

Sporbarhet for hva som er gjenbrukt, kopiert og generert. Holder
rapportens systembeskrivelse aerlig om hvor mye nytt arbeid pakka
representerer.

| Fil | Lab4 | usb_cam-eksempel | Generert | Verifisert via testing |
|---|---|---|---|---|
| `package.xml` | Struktur og format-skjelett | Avhengighetslista utvidet med usb_cam, vision_msgs | Beskrivelse, maintainer, MIT-lisens | — |
| `setup.py` | Komplett mal | — | Beskrivelse, kommentert entry_points | — |
| `setup.cfg` | Identisk format | — | Kun byttet pakkenavn | — |
| `config/camera_params.yaml` | — | `pixel_format`, `av_device_format`, `-1`-moenster, navnene `autofocus`/`autoexposure`/`auto_white_balance` | `video_device` (by-id-symlink), 1280x720@30fps, `frame_id`, `camera_name`, alle norske kommentarer | Krasj uten `av_device_format`; v4l2-kontrollnavn-mismatch paa Ubuntu 24.04 |
| `launch/camera.launch.py` | `Node()` + `PathJoinSubstitution`-moenster fra `pipeline.launch.py` | — | `resolve_camera_device()` (workaround for usb_cam-symlink-bug), `CAMERA_BY_ID`-konstant | usb_cam tolket relativ symlink-target bokstavelig |
| `README.md` | — | — | Hele filen | — |
| `resource/vision`, `vision/__init__.py` | Standard ament_python tomme markorfiler | — | — | — |

`find_object_2d` ble vurdert som alternativ deteksjonsmetode men ikke
brukt — solide ensfargede kuber har for faa feature-punkter til at
ORB/SIFT/SURF gir paalitelig deteksjon. Det blir et metodevurderings-
argument i rapportens Systemtenkning-kapittel, ikke en kodekilde.
