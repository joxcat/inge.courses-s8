# SADT student

## Course instructions

Look at the [wiki](https://gitlab.com/m0rph03nix/sadt_student/-/wikis/home)

## Files description

`bin/vehicle_checker` binary file sends and gets CAN frame to test the avsim2D vehicle with the following behavior :

[!avsim2D](https://gitlab.com/m0rph03nix/sadt_student/-/wikis/img/avsim2D_check.gif)

## TP1
- ID: `0x3321`
	- `640002` => turn left fast (big turn)
	- `640009` => turn left fast (small turn)
	- `100002` => turn left slow (big turn)
	- `100009` => turn left slow (small turn)
	- `100000` => forward slow
	- `002000` => brakes
- ID: `0x2123`
	- `0001` => clignotants droits
	- `0002` => clignotants gauches
	- `0100` => feux de croisement
	- `0200` => feux de route
	- `0000` => pas de feux

### ID 0x3321
AABBCC
AA => vitesse entre 0 et 100 (0x64)
BB => brakes entre 0 et 100 (0x64)
CC => rotation entre -100 (0x9C -> +100 tourner à droite) et 100 (tourner à gauche)

## TP2
### For each area, describe what is seen by the vehicle
#### Full left => route
In the full left (ID 0x80000C00), the vehicle sees:
- the road: 0x50 (80)
- yield marking: 0x00 (0)
- road crossing: 0x00 (0)
- stop sign: 0x00 (0)
- car park: 0x16 (22)

#### Left => route
In the left (ID 0x80000C01), the vehicle sees:
- the road: 0x5B (91)
- yield marking: 0x00 (0)
- road crossing: 0x00 (0)
- stop sign: 0x00 (0)
- car park: 0x0B (11)

#### Middle left => route
In the middle left (ID 0x80000C02), the vehicle sees:
- the road: 0x5A (90)
- yield marking: 0x00 (0)
- road crossing: 0x00 (0)
- stop sign: 0x00 (0)
- car park: 0x09 (9)

#### Middle right => route
In the middle right (ID 0x80000C03), the vehicle sees:
- the road: 0x5A (90)
- yield marking: 0x00 (0)
- road crossing: 0x00 (0)
- stop sign: 0x00 (0)
- car park: 0x09 (9)

#### Right => route
In the right (ID 0x80000C04), the vehicle sees:
- the road: 0x5A (90)
- yield marking: 0x00 (0)
- road crossing: 0x00 (0)
- stop sign: 0x00 (0)
- car park: 0x0B (11)

#### Full right => route
In the full right (ID 0x80000C05), the vehicle sees:
- the road: 0x4F (79)
- yield marking: 0x00 (0)
- road crossing: 0x00 (0)
- stop sign: 0x00 (0)
- car park: 0x17 (23)


