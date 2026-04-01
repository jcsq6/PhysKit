# Pool Demo Tuning

Tunable parameters are exposed at the top of `tests/pool/pool_demo.cpp`. The most impactful parameters:

- `BALL_RADIUS` — visual and collision radius for balls. Increasing will change spacing in racks and collision response.
- `BALL_MASS` — mass of each ball; affects momentum in break shots.
- `BALL_RESTITUTION` — bounciness of balls; higher values make more lively collisions.
- `BALL_FRICTION` — contact friction between balls and table/rails; low values let balls slide more before rolling.
- `CUE_IMPULSE_MULT` — scalar used to convert charge power into a desired ball speed; change to tune shot responsiveness.
- `POCKET_RADIUS` — how close the ball center must be to a pocket center to be considered sunk.
- `VELOCITY_THRESHOLD` — below this speed (m/s) balls are considered settled and player may shoot again.
- `SOLVER_ITERATIONS` — increase in `g_config` (or JSON) if the break shows instability; higher values cost CPU but give more robust contact resolution.

Suggested starting values (used in the demo):

- `BALL_RADIUS = 0.05715 m` (standard pool ball radius ~57 mm)
- `BALL_MASS = 0.17 kg`
- `BALL_RESTITUTION = 0.95`
- `BALL_FRICTION = 0.05`
- `CUE_IMPULSE_MULT = 1.5` (tweak to taste)
- `POCKET_RADIUS = BALL_RADIUS * 2.2`
- `VELOCITY_THRESHOLD = 0.03 m/s`

When tuning, change one parameter at a time and observe the result — solver iterations and restitution interact nonlinearly with break shot energy.
