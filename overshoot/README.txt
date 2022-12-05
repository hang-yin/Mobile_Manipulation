Controller: Feedforward-plus-PI
Kp gain: 2.0
Ki gain: 50.0

For the overshoot condition, I experimented with a Feedforward-plus-PI controller. 
I started experimenting with large Kp gains and small Ki gains but they didn't seem to produce the desired behavior.
I eventually decided on using a very large Ki gain and a small Kp gain, which produce the bahavior shown in the video. 

Initial Configuration: 
chassis phi - 0.8
chassis x - -0.4
chassis y - 0.25
J1 - 0.133
J2 - -0.338
J3 - -0.474
J4 - -0.674
J5 - 0.0
W1 - 0.0
W2 - 0.0
W3 - 0.0
W4 - 0.0
gripper state - 0