# Temporal Logic Primer

Each scenario in IITCHS must have a temporal logic (TL) formula which specifies the mission requirements for the agent teams.
This document gives a brief overview of how to write custom formulas for scenarios.

## Capability Temporal Logic

IITCHS discretizes time into time steps (e.g. `t=0, 1, 2, ....`). At each time step, we may one or more desired conditions to be
satisfied by a multi-agent team. In addition, we may want desired conditions to hold over one or more time intervals. For example,
* At the next time step, have 3 agents with cameras in area A
* Have an agent with a Lidar unit visit area B at least once every 30 minutes.
* Within the next hour, an agent with both a Lidar and IR sensor should arrive in area D and stay there indefinitely

IITCHS uses a fragment of Temporal Logic called **Capability Temporal Logic** (CaTL) to express these types of requirements
for specific conditions called **tasks**. 

### Tasks
A task is made of three elements:
* A time duration (number of time steps; e.g. 10 time steps)
* A state or state label (e.g. state A, or state label 'orange')
* One or more counting propositions (explained below).

Tasks are denoted by `T(...)` (examples below). 

A counting proposition is a tuple containing two elements:
* One agent capability
* A desired number of agents

To give an example, suppose some of our agents have visual cameras (`VIS`), some have Lidar units (`LID`), and some have IR (`IR`) sensors. The list of possible capabilities would be `VIS, LID, IR`. 
Some examples of counting propositions would then include:
* `(IR,4)` -- Four agents with IR sensors. 
* `(VIS, 1)` -- One agent having a visual camera.

Continuing our example, suppose we have four states `A, B, C, D`. States `A, C` each are labeled `blue`, and states `B, D` are each labelled `red`. Then some examples of tasks would include:
* `T(5, A, {(VIS, 1)})` -- 1 agent with the `VIS` capability present in state A for 5 consecutive time steps
* `T(15, red, {(LID, 3)}` -- 3 agents with the `LID` capability simultaneously present in each state labelled `red` (i.e. states `B` and `D`) for 15 consecutive time steps
* `T(18, blue, {(LID, 2), (IR,6)})` -- 2 agents with the `LID` capability and 6 agents with the `IR` capability present in the `blue` labelled states for 18 consecutive seconds.

A task with duration `d` is "satisfied" at time `t` if its specified conditions are true for all time steps in `[t,t+d]`. For example, the task `T(5, D, {(VIS,3)})` is satisfied at time `t` if (and only if) state D contains at least three agents with the `VIS` capability for all time steps in the interval `[t, t+5]`. When a task contains a state label (e.g. `blue` or `red`), the conditions must hold _for all states with that label_ in order for the task to be satisfied. As a further example, the task `T(7, red, {(IR,2)})` is satisfied if (and only if) all red states each contain at least 2 agents with the `IR` capability for all time steps in `[t, t+7]`.

Finally, note that it is possible for some agents to have multiple capabilities; e.g. an agent could have both a visual camera and an IR sensor. IITCHS takes into account each agent's individual set of capabilities when creating a plan to satisfy all required tasks. Having agents with multiple capabilities simply gives the IITCHS algorithm more flexibility in creating a final plan.

### Temporal (Time) Operators

Temporal operators are used to specify time-dependent requirements such as "Satisfy task X within the next 30 time steps".
The three temporal operators are:
* `G[a,b]` -- "Always" (time-bounded)
* `F[a,b]` -- "Eventually" (time-bounded)
* `U[a,b]` -- "Until" (time-bounded)

Given a task `T(...)` and a time `t0`, the formula `G[a,b] T(...)` holds true at `t0` if (and only if) the task `T(...)` is satisfied at all time steps in the interval `[t0+a, t0+b]`.
As an example, for the formula `G[2,6] T(3, B, {(LID,2)})` to hold true at time `t0`, there would need to be at least 2 agents with capability `LID` in state `B` for all time steps in the interval `[t0+2, t0+9]`. Notice that the end of the interval is `t0+9` rather than `t0+6`. This is because the task duration is 3 time steps long, implying that the conditions need to be satisfied from `t0+2` to `t0+6+3 = t0+9`.


Given a task `T(...)` and a time `t0`, the formula `F[a,b] T(...)` holds true at `t0` if (and only if) there exists at least one time step `t*` in the interval `[t0+a, t0+b]` where `T(...)` is satisfied. 
As an example, for the formula `F[5,50] T(4, blue, {(VIS, 1)})` to hold true at time `t0`, there would need to be at least one time step `t` between 5 and 50 such that each blue-labelled state contained one agent with the `VIS` capability for the interval `[t, t+4]`. If each blue labelled state contained one agent with the `VIS` capability for all times in the interval `[26,30]`, the formula would be satisfied.


Given two tasks `T1(...)` and `T2(...)` and a time `t0`, the formula `T1(...)U[a,b]T2(...)` holds true if (and only if) there exists a time `t*` in the interval `[t0+a, t0+b]` where `T2(...)` is satisfied and for all time steps `t**` in the interval `[t0+a,t*-1]` `T1(...)` is satisfied. In other words, starting at time `t0+a`, `T1(...)` must be satisfied *until* `T2(...)` is satisfied, and `T2(...)` must be satisfied by time `t0+b`. As an example, for the formula `T(1,blue,{VIS,1})U[5,10]T(3,red,{IR,2})` to hold true at time 0, there must be some time step `t` between 5 and 10, such that each red-labelled state contained two agents with the `IR` capability for the interval `[t,t+3]`. At every time step in the interval `[5,t-1]` there must be at least one agent with the `VIS` capability or each one time-step interval. If each red-labelled state contained two agents with the `IR` capability for all times in the interval `[7,8,9]` and each blue-labelled state contained one agent with the `IR` capability for all times in the interval `[5,6]`, the formula would be satisfied.


The operators `F`, `G`, and `U` can be combined in sequence:
* `G[a,b] F[c,d]` -- "Always eventually" operator. Used for recurring tasks, e.g. `G[0,100] F[0,10] T(1, B, {(IR,1)})` means "Over a time window of 100 time steps, an agent with the `IR` capability must visit state `B` one out of every 10 time steps".
* `F[a,b] G[c,d]` -- "Eventually always" operator. Used for ensuring agents eventually visit a state/states and remain there.


### Limit Operator

The Limit operator allows creating upper bounds on the number of capabilities within a given state or set of states. The Limit operator is denoted by `L(...)` and consists of two elements:
* A state or state label
* A counting proposition

To give some examples, 
* The formula `L(B, {(LID, 2)})` is satisfied at time `t` if (and only if) there are no more than 2 agents with the `LID` capability in state `B`.
* The formula `L(blue, {(IR, 3), (VIS, 2)})` is satisfied at time `t` if (and only if) there are no more than 3 agents with the `IR` capability and no more than 2 agents with the `VIS` capability in each of the blue-labelled states at time `t`.

Limit operators can be combined with all of the operators previously discussed:
* `G[a,b] L(C, {(VIS, 1)})` -- For all time steps in the interval `[t0+a, t0+b]`, there must be no more than one agent with the `VIS` capability in state `C`.
* `F[a,b] G[0,d] L(red, {(LID,5)})` -- There must eventually exist a time step `t` in the interval `[t0+a, t0+b]` such that at least 5 agents with the `LID` capability are in each of the red-labelled states for `d` time steps (i.e. the interval `[t+0, t+d]`).


### Boolean Logic

Smaller CaTL formula can be combined together into larger formula using the Boolean operators AND (`&&`) and OR (`||`). For example, `T(4, A, {(IR, 1)} || T(2, B, {(VIS,3)})` means that either the first or second task should be satisfied (at a given time `t`).

Note that the Boolean operator NOT (&not;) is not natively included in CaTL. This is because negation of a formula requires being able to negate a task. There are many ways to negate a task (not enough agents, not enough duration, etc.). If we define the Limit operator as the negation of a task, then NOT can be included in CaTL, as well as any other related logical operators (e.g., Implication (A&rArr;B), equivalent to &not;(A&&&not;B)) can be included. **This is not yet implemented.**

## Example Formulas

**Example 1:**
```
F[0, 35] T(3, A, {(A1,3)}) && F[0, 30] T(3, B, {(A1,3)}) && F[0,30] T(5, C, {(A1,3)}) && F[35,60] T(5, A, {(A2,3)}) && F[30,60] T(5, B, {(A2,3)}) && F[15,45] T(5, C, {(A2,3)})
```
This consists of 6 subformulas all combined together with the AND operator.
* `F[0, 35] T(3, A, {(A1,3)})`: Within the first 35 time steps (the initial time t0 is implied to be zero), three agents with capability `A1` must simultaneously visit state `A` for at least three consecutive time steps.
* `F[0, 30] T(3, B, {(A1,3)})`: Within the first 30 time steps, three agents with capability `A1` must simultaneously visit state `B` for at least three consecutive time steps.
* `F[0,30] T(5, C, {(A1,3)})`: Within the first 30 time steps, three agents with capability `A1` must simultaneously visit state `C` for at least five consecutive time steps.
* `F[35,60] T(5, A, {(A2,3)})`: Between time steps 35 and 60 (the initial time t0 is implied to be zero), three agents with capability `A2` must simultaneously visit state `A` for at least five consecutive time steps.
* `F[30,60] T(5, B, {(A2,3)})`: Between time steps 30 and 60, three agents with capability `A2` must simultaneously visit state `B` for at least five consecutive time steps.
* `F[15,45] T(5, C, {(A2,3)})`: Between time steps 15 and 45, three agents with capability `A2` must simultaneously visit state `C` for at least five consecutive time steps.
