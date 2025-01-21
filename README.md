Lead Screw = 2mm

Desired Screw Pitch = 1.75

Stepper Motor Steps/Revolution = 200

Rottary Encoder Pulses/Revolution = 4000


Rlead Screw = RSplindle * (Desire Pitch/Lead Screw Pitch)

Rlead Screw = Rspindle * (1.75/2)

Rlead Screw = 200 * 0.875

Rlead Screw = 175 Steps (per one revolution of Tsoc)

Rottary Encoder Steps = 4000/175 = 22.857


0.875 from 22.857 is 6/7
(22*6)/7 = 160/7

so i have to divide 160 pulses to 7 slots. 
int numbers175[] = {22,23,23,24,23,23,22};

<!-- 175/1 = 175 X
175/2 = 87.5 X
175/3 = 58.33333 X
175/4 = 43.75 X
175/5 = 35 X
175/6 = 29.1666 X
175/7 = 29.1666 X -->
