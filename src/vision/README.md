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
| `hsv_tuner` | Klar | OpenCV-trackbars for live tuning av HSV-grenser per farge. Lagrer til `config/hsv_thresholds.yaml`. |
| `hsv_detector` | Klar | Subscriber `/image_raw`. Publiserer `/vision/detections` (`vision_msgs/Detection2DArray`, piksel-posisjon) + annotert debug-bilde `/vision/image_annotated`. |
| `detection_logger` | Planlagt | Logger deteksjoner til CSV med tidsstempel + lysforhold. |

## Kjoring

`hsv_detector` krever meldingspakken `vision_msgs` (engangsinstall):

```bash
sudo apt install ros-jazzy-vision-msgs
```

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

### Tune HSV-grenser per farge

Kjoer med kameraet i gang. Dra trackbars til kuben isoleres rent i
maske-vinduet, trykk `s` for aa lagre, `q`/ESC for aa avslutte:

```bash
ros2 run vision hsv_tuner --ros-args -p color:=blue \
    -p output_file:=src/vision/config/hsv_thresholds.yaml
```

Roed farge wrapper rundt H=0/180 - tune de to omraadene hver for seg
(eller rediger `hsv_thresholds.yaml` for haand, se kommentarene der).

### Kjoer deteksjonen

Hele fase-1 pipelinen (kamera + detektor) i én kommando:

```bash
ros2 launch vision detect.launch.py            # evt. min_area:=800
```

Inspiser resultatet:

```bash
ros2 run rqt_image_view rqt_image_view         # velg /vision/image_annotated
ros2 topic echo /vision/detections             # piksel-posisjon per kube
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
| ROS2 + OpenCV-integrasjon | automaticaddison, "Getting Started With OpenCV in ROS 2", https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/ (Foxy, men moenster er versjonsuavhengig) |
| OpenCV HSV-fargerom (HSV-metoden) | OpenCV docs, "Changing Colorspaces", https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html |
| OpenCV konturanalyse (bbox + sentroide) | OpenCV docs, "Contour Features", https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html |
| Deteksjons-meldingsformat | vision_msgs, https://github.com/ros-perception/vision_msgs |
| Systemoppbygging (referanse) | SMARTlab-Purdue, "Robot control with colored object detection", https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki (ROS1 - konseptuell, ikke kodekilde) |
| Klassisk maskinsyn | Bradski & Kaehler, *Learning OpenCV 3*, O'Reilly (2017) |
| Generelt | Szeliski, *Computer Vision: Algorithms and Applications*, 2. utg. (2022) |
| Vurdert og forkastet | Labbe, find_object_2d, http://wiki.ros.org/find_object_2d (ROS1-wiki; ROS2-port `introlab/find-object`). Krever teksturerte objekter. |
| Tidligere arbeid | Lab4 camera_pipeline (AIS2105, varsemester 2026) |

## Opphav per fil

Sporbarhet for hva som er gjenbrukt, kopiert og generert. Holder
rapportens systembeskrivelse aerlig om hvor mye nytt arbeid pakka
representerer.

| Fil | Lab4 | Ekstern kilde | Generert | Verifisert |
|---|---|---|---|---|
| `package.xml` | Struktur og format-skjelett | Avhengighetslista utvidet med usb_cam, vision_msgs, python3-yaml | Beskrivelse, maintainer, MIT-lisens | — |
| `setup.py` | Komplett mal | — | Beskrivelse, entry_points for hsv_tuner/hsv_detector | colcon build OK |
| `setup.cfg` | Identisk format | — | Kun byttet pakkenavn | — |
| `config/camera_params.yaml` | — | `pixel_format`, `av_device_format`, `-1`-moenster, navnene `autofocus`/`autoexposure`/`auto_white_balance` (usb_cam-eksempel) | `video_device` (by-id-symlink), 1280x720@30fps, `frame_id`, `camera_name`, alle norske kommentarer | Krasj uten `av_device_format`; v4l2-kontrollnavn-mismatch paa Ubuntu 24.04 |
| `launch/camera.launch.py` | `Node()` + `PathJoinSubstitution`-moenster fra `pipeline.launch.py` | — | `resolve_camera_device()` (workaround for usb_cam-symlink-bug), `CAMERA_BY_ID`-konstant | usb_cam tolket relativ symlink-target bokstavelig |
| `config/hsv_thresholds.yaml` | — | Roede startverdier fra testlogg 20260512-09 | Filformat (multi-range per farge), gul/blaa startgjett, kommentarer | Roed verifisert mot fysisk kube under skygge (forrige oekt); gul/blaa maa tunes |
| `vision/hsv_tuner.py` | cv_bridge-subscriber + node-moenster | `cvtColor`+`inRange` fra OpenCV "Changing Colorspaces"-tutorial | Trackbar-GUI, YAML-lagring, keypress-haandtering | py_compile + colcon build OK; ikke kjoert mot kamera ennaa |
| `vision/hsv_detector.py` | cv_bridge sub/pub + node-moenster | HSV-maske (OpenCV colorspaces), `findContours`/`boundingRect`/`moments` (OpenCV "Contour Features"); `vision_msgs`-felt-API verifisert mot ros-perception/vision_msgs | Morfologi (open/close), multi-range-OR for roed, Detection2DArray-mapping, bilde-annotering | py_compile + colcon build OK; ikke kjoert mot kamera ennaa |
| `launch/detect.launch.py` | `Node()`-moenster | `IncludeLaunchDescription` - standard ROS2-launchmoenster | Sammensetning kamera+detektor, `min_area` launch-argument | py_compile + colcon build OK |
| `README.md` | — | — | Hele filen | — |
| `resource/vision`, `vision/__init__.py` | Standard ament_python tomme markorfiler | — | — | — |

`find_object_2d` ble vurdert som alternativ deteksjonsmetode men ikke
brukt — solide ensfargede kuber har for faa feature-punkter til at
ORB/SIFT/SURF gir paalitelig deteksjon. Det blir et metodevurderings-
argument i rapportens Systemtenkning-kapittel, ikke en kodekilde.
