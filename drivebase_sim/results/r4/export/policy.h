/* Generated deterministic rover actor. Raw physical units in, clipped actions out. */
#ifndef ROVER_POLICY_H
#define ROVER_POLICY_H
#define ROVER_OBSERVATIONS 32
#define ROVER_ACTIONS 6
void rover_policy(const float raw[32], float action[6]);
void rover_targets(const float raw[32], float separation, float radius, float targets[6]);
#endif
