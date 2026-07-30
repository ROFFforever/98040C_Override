USER PROFILE:

\*The user is from team: 98040C. And their name is: Brian

\*User has uploaded both their repo(Slippery Penguins(team number: 98040C)) and another team's repo(echo\_code).

\* The user is analyzing/taking inspiration off an external VEX VRC V5 team's repository (not their own). This team's name is: Echo.

\* User is most comfortable in Java, and is still learning about C++. However they are limited only to AP Computer science level Java(so more advanced concepts, templates, advanced lambdas, etc... are unknown yet).

\* Explain complex C++ idioms (pointers, references, memory allocation, macros) simply using clear Java mental models, AVOIDING overly advanced or dense C++ jargon.

\*User prioritizes clear, comprehensive, "from the beginning" typed explanations compared to jargon or contextually advanced idioms within cs.

\*User is pretty much a beginner in ADVANCED robotics programming

\*User DOES NOT come from WLib or FRC.

\*When explaining issues, try to "sum up" the problem rather than diving deep into the technical end and then failing to provide a quick "TLDR". A deep technically dive is OK, as long as a TLDR is given, remember, user is not a super user.

\*Be truthful with the user. If you require more context or logs, say it, don't just go off making some random improvement based off assumption.

\*When user mentions something in their project, assume it is the 98040C folder.

\*Don't write comments or edit user written comments

CONTEXT:
98040C is user's current project. 

OLD\_CODE is the user's past codebase from last year

Echo\_code is a model example of what the user aspires to be. Echo\_code contains model Ramsete implementation, localization(MCL), and command scheduler framework, and more.

Slippery Penguins contains a stock copy of Lemlib, which contains it's own rich set of utilizes.(Timers, PID, robust odom, clean/intuitive classes, and Pure Pursuit).


USER's GOAL:
They want to create a more organized, accurate(localization wise), and better autonomous routes/movements(maybe path following vs chained boomerang + simple PID motions)

They want to take inspiration and model off of Echo.



### 6. Environment / Build Notes
- `pros build` (or raw `make`) run inside Claude Code's sandboxed Bash tool fails at the final link stage with `cannot execute 'cc1'`. Confirmed (via Brian's own terminal output) this is **sandbox-specific**, not a real project/toolchain problem — full `pros build` succeeds cleanly for Brian locally. Standing approach for future sessions: verify each touched `.cpp` file compiles individually (`pros build` reports `Compiled <file> [OK]` per file before hitting the link-stage error) and let Brian run the full build himself to confirm final link/upload. (Also saved in Claude's persistent memory as `build_cc1_sandbox_issue.md`.)
