Constants:

Lead Screw = 2mm

Desired Screw Pitch = 1.75

Stepper Motor Steps/Revolution = 200

Rottary Encoder Pulses/Revolution = 4000

***************************************************************** sasasasas

Calculations:

Ratio = Desire Pitch/Lead Screw Pitch

Rlead Screw = RSplindle * (Desire Pitch/Lead Screw Pitch)

Rlead Screw = Rspindle * (1.75/2)

Rlead Screw = 200 * 0.875

Rlead Screw = 175 Steps (per one revolution of Chuck)

Rottary Encoder Pulses/Step = 4000/175 = 22.857


0.875 from 22.857 is 6/7
22 + 0.857 = 22 + 6/7 = (22*7)/7 + 6/7 = 154/7 + 6/7 = 160/7

so i have to divide 160 pulses to 7 slots. 
int numbers175[] = {22,23,23,24,23,23,22};




The method we’ve shown:

    Take two integers: pulses and steps.
    Compute their GCD: gcd⁡(pulses,steps).
    Divide:
    pulses/gcd⁡(pulses,steps) and steps/gcd⁡(pulses,steps).
    You end up with an integer fraction in simplest form.

This process is universal for any two integers you want to represent as a ratio. It doesn’t matter whether you have 4000 pulses or 2000 pulses (or 50,000 pulses)—the same exact steps apply.