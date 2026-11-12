# SSS URC rover model: parameters and where they come from

Checked 2026-11-08. "Team" means the team's own rover design, whose documents are not republished here; "assumed" means no source yet (replace when
a CAD model or a measurement is available).

## From the team

Chassis geometry, materials, wheels and loads follow the team's rover design documents. Their details are not
reproduced here, so every value in the model that is not in the tables below is an assumption.

| Item | Value used | Source |
|---|---|---|
| Suspension | rocker-bogie, six wheels, one motor per wheel | the team's rover design |
| Drive motors | 6 x DM-J6006-2EC, integrated driver and encoder, CAN 1 Mbps | the team's rover design; manufacturer manual |
| Motor data | 76 mm x 36.5 mm, 335 g, 24 V (24-48 V), 4 N m rated, 12 N m peak, 150 rpm rated, 240 rpm no-load, 4 A rated, 13 A peak, 6:1 | Damiao parameter table (listed next to DM-J4310; the J6006 column matches its 76 x 36.5 mm, 335 g, 6:1) and retailer listings; manual V1.0 for 24 V and CAN modes |
| Control board | SSS Rover_Control_Board_v1 @ 5db1aab, exported from KiCad | GitHub |

## Assumed

| Item | Value | Why |
|---|---|---|
| Wheel | 200 mm diameter, 80 mm wide | typical for a 50 kg-class URC rover on DM-J6006 hub motors; photo shows a tyre much wider than the motor |
| Track, wheelbase | about 0.72 m between wheel centres (follows from the plate, bracket, motor and rim stack), 0.84 m front to rear | assumed proportions of a rocker-bogie rover |
| Electronics box | 0.60 x 0.42 x 0.20 m | the chassis team's housing is not public |
| Masses | chassis with electronics 38 kg, rocker 2.2 kg, bogie 2.6 kg, wheel 0.9 kg (total about 53 kg, a typical 50 kg-class rover) | assumed |
| Battery | 24 V pack | matches the motors' rated voltage and the drivebase doc's "24 V -> 5 V" buck |
| Wheel-soil friction | 0.9 sliding, 0.01 torsional | dry desert soil |
| Camera mast, e-stop, GNSS | shapes only | URC rovers carry them; positions are illustrative |
