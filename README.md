## Robotarm for palletering – ELE208 Robotmanipulatorer (HVL, høst 2026)

6-DOF robotarm som plukker esker fra et transportbånd og stabler de i to lag
på en pall. Modellert i MATLAB med Peter Corke's Robotics Toolbox og
simulert i Gazebo via ROS 2.

Gruppe: Christoffer Træet Herland, Sigurd Vikingstad Hodne,
Ingrid Soldal Vikane, Erlend Østbø

## Videoer
- Palletering av 12 esker i MATLAB: https://youtu.be/s24GQhQxH6w
- Kamerastyrt plukk av roterte esker: https://youtu.be/qtbNQoKE5i4

## Krav til kjøring av kode
- MATLAB med Peter Corke's Robotics Toolbox
- ROS 2 og Gazebo (Ubuntu) for simuleringen

## Filer og rekkefølge
Filer og beskrivelse

    'Palletizing_Robotarm.m'                   /   Robotmodell (DH-parametere, leddgrenser, TCP)

    'Workspace.m'                              /   Arbeidsområdet: transportbånd, pall, kamera, eske i MATLAB

    'Forward_kinematics_prosjekt_for_hand.m'   /   T04 utledet for hånd, sammenlignet med toolbox 

    'InverseKinematics_Palletizing12boxes.m'   /   Baneplanlegging for 12 esker -> lagrer 'pallet_cycles.mat' som brukes videre

    'Animasjon.m`                              /   Animasjon og video av banene 

    'differentialkinematic.m`                  /   Leddhastigheter, akselerasjoner og TCP-fart 

    'Palletizing_Dynamics_v3.m`                /   Dynamisk modell og rne()

    'Palletizing_Dynamics_test.m'              /   Kontroll av aktuatorvalg

    'PosisjonsKontroller.m'                    /  Simulert posisjons kontroller til ledd 2

    'ros2/...'                                 /   URDF, Gazebo-oppsett, 'run_joint_trajectory.m' 

NB! Kjør først baneplanleggingen. De påfølgende scriptene leser 'pallet_cycles.mat'.

## Bruk av KI
Beskrevet i rapporten.
