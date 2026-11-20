"""Two-body identification in physical seconds/effort/position units.

Optional SciPy dependency. Same dead-zone spring + always-active damper law as
simmy; this is not a contact/impact or hysteretic backlash model.
"""
import numpy as np


NAMES = ['motor_inertia', 'load_inertia', 'motor_viscous', 'load_viscous',
         'motor_bias', 'load_bias', 'stiffness', 'damping']


def deadzone(relative, gap):
    """gap is half-width, as in SimmyCouplingParams.backlashGap."""
    return np.sign(relative) * np.maximum(np.abs(relative)-gap, 0.)


def blocks(rows):
    start = 0
    while start < len(rows):
        end = start+1
        while end < len(rows) and rows[end]['segment'] == rows[start]['segment']:
            end += 1
        yield rows[start:end]
        start = end


def prepare(rows, window):
    windows = []
    for block in blocks(rows):
        if len(block) < 5:
            continue
        t, x, y, effort = (np.array([float(r[key]) for r in block]) for key in
                            ('time', 'motor_position', 'load_position', 'effort'))
        vx, vy = np.gradient(x, t, edge_order=2), np.gradient(y, t, edge_order=2)
        i = 1
        while i < len(t)-2:
            j = int(np.searchsorted(t, t[i]+window))
            if j >= len(t)-1:
                break
            sl = slice(i, j+1)
            weights = np.zeros(j-i+1)
            weights[:-1] += np.diff(t[sl])/2
            weights[1:] += np.diff(t[sl])/2
            windows.append((block[0]['segment'], vx[j]-vx[i], vy[j]-vy[i],
                            x[j]-x[i], y[j]-y[i], t[j]-t[i],
                            x[sl]-y[sl], weights, float(weights@effort[sl])))
            i = j
    return windows


def matrix(windows, gap):
    a, b = [], []
    for _, dvx, dvy, dx, dy, dt, relative, weights, effort in windows:
        spring = float(weights@deadzone(relative, gap))
        a.extend([[dvx, 0, dx, 0, dt, 0, spring, dx-dy],
                  [0, dvy, 0, dy, 0, dt, -spring, dy-dx]])
        b.extend([effort, 0.])
    return np.array(a), np.array(b)


def fit_compliance(rows, window, max_gap):
    from scipy.optimize import lsq_linear, minimize_scalar

    if not np.isfinite(max_gap) or max_gap <= 0 or window <= 0:
        raise ValueError('window and max-gap must be positive')
    if any('load_position' not in r or not np.isfinite(float(r['load_position'])) for r in rows):
        raise ValueError('two-body fitting requires finite load_position on every sample')
    train_rows = [r for r in rows if r['segment'] in ('friction', 'acceleration', 'dynamic')]
    relative = np.array([float(r['motor_position'])-float(r['load_position']) for r in train_rows])
    if len(relative) == 0 or relative.min() >= 0 or relative.max() <= 0:
        raise ValueError('training must excite both signs of calibrated relative displacement')
    windows = prepare(rows, window)
    train = [w for w in windows if w[0] in ('friction', 'acceleration', 'dynamic')]
    valid = [w for w in windows if w[0] == 'validation']
    if len(train) < 16 or len(valid) < 4:
        raise ValueError('need training and separate validation windows')

    def solve(gap):
        a, b = matrix(train, gap)
        scale = np.linalg.norm(a, axis=0)
        if np.any(scale < 1e-15):
            return None
        scaled = a/scale
        singular = np.linalg.svd(scaled, compute_uv=False)
        if singular[-1] < singular[0]*1e-10:
            return None
        lower = np.array([1e-12, 1e-12, 0, 0, -np.inf, -np.inf, 1e-12, 0])
        result = lsq_linear(scaled, b, bounds=(lower*scale, np.full(8, np.inf)))
        if not result.success:
            return None
        theta = result.x/scale
        return float(np.sum((a@theta-b)**2)), theta, float(singular[0]/singular[-1])

    # Profile the non-smooth gap rather than entrusting a single local start.
    grid = np.linspace(0, max_gap, 41)
    candidates = [(float(g), solve(float(g))) for g in grid]
    finite = [(g, result) for g, result in candidates if result is not None]
    if not finite:
        raise ValueError('unidentifiable two-body model: insufficient independent excitation')
    best_gap, best = min(finite, key=lambda item: item[1][0])
    index = int(np.argmin(np.abs(grid-best_gap)))
    lo, hi = grid[max(0, index-1)], grid[min(40, index+1)]
    def objective(gap):
        result = solve(gap)
        return result[0] if result is not None else 1e100
    refined = minimize_scalar(objective, bounds=(lo, hi), method='bounded',
                              options={'xatol': max_gap*1e-7})
    result = solve(refined.x)
    if result is not None and result[0] < best[0]:
        best_gap, best = float(refined.x), result
    _, theta, condition = best
    if relative.min() >= -best_gap or relative.max() <= best_gap:
        raise ValueError('fitted gap lacks engagement on both sides')
    parameters = dict(zip(NAMES, theta.tolist()))
    parameters['backlash_half_gap'] = best_gap
    warnings = ['OLS on measured positions can be biased by noise and feedback.',
                'No Coulomb/stiction model; always-active coupling damper matches simmy.',
                'Requires calibrated relative position zero and common mechanical coordinates.']
    if best_gap >= .98*max_gap:
        warnings.append('Gap at search boundary: increase max-gap and repeat the fit.')
    if condition > 1000:
        warnings.append('Poor conditioning: individual parameters may be ambiguous.')
    zero = solve(0.)
    improvement = (1-best[0]/zero[0]) if zero is not None and zero[0] > 0 else None
    if improvement is not None and improvement < .05:
        warnings.append('Little evidence for nonzero backlash versus the zero-gap model.')
    report = dict(model='two_body_deadzone_integral', parameters=parameters,
                  window_seconds=window, max_gap=max_gap, scaled_condition=condition,
                  warnings=warnings, gap_profile=[dict(half_gap=g, squared_error=r[0] if r else None)
                                                  for g, r in candidates],
                  training_sse_without_gap=zero[0] if zero else None,
                  gap_sse_improvement_fraction=improvement)
    for label, subset in [('training', train), ('validation', valid)]:
        a, b = matrix(subset, best_gap)
        error = a@theta-b
        report[label] = dict(windows=len(subset), effort_integral_rmse=float(np.sqrt(np.mean(error**2))),
                             relative_error=float(np.linalg.norm(error)/max(np.linalg.norm(b), 1e-15)))
    predictions = predict_validation(rows, parameters)
    for coordinate in ('motor', 'load'):
        error = np.array([r[f'{coordinate}_predicted']-r[f'{coordinate}_position'] for r in predictions])
        report['validation'][f'{coordinate}_position_rmse'] = float(np.sqrt(np.mean(error**2)))
        report['validation'][f'{coordinate}_position_max_error'] = float(np.max(np.abs(error)))
    return report, predictions


def derivative(state, effort, p):
    x, vx, y, vy = state
    force = p['stiffness']*deadzone(x-y, p['backlash_half_gap']) + p['damping']*(vx-vy)
    return np.array([vx, (effort-force-p['motor_viscous']*vx-p['motor_bias'])/p['motor_inertia'],
                     vy, (force-p['load_viscous']*vy-p['load_bias'])/p['load_inertia']])


def predict_validation(rows, parameters):
    """Open-loop effort replay, one initial state per held-out block; no position resets."""
    from scipy.integrate import solve_ivp

    result = []
    for block in blocks(rows):
        if block[0]['segment'] != 'validation' or len(block) < 5:
            continue
        t, x, y, effort = (np.array([float(r[key]) for r in block]) for key in
                            ('time', 'motor_position', 'load_position', 'effort'))
        vx, vy = np.gradient(x, t, edge_order=2), np.gradient(y, t, edge_order=2)
        state = [x[0], vx[0], y[0], vy[0]]
        solution = solve_ivp(lambda time, s: derivative(s, np.interp(time, t, effort), parameters),
                             (t[0], t[-1]), state, t_eval=t, rtol=1e-7, atol=1e-10,
                             max_step=float(np.max(np.diff(t))))
        if not solution.success or not np.all(np.isfinite(solution.y)):
            raise ValueError('held-out trajectory integration failed')
        result.extend(dict(time=float(time), segment='validation', motor_position=float(x[i]),
                           load_position=float(y[i]), motor_predicted=float(solution.y[0, i]),
                           load_predicted=float(solution.y[2, i])) for i, time in enumerate(t))
    if not result:
        raise ValueError('no held-out trajectory to predict')
    return result


def demo_compliance(plan, gap=.002):
    """Independent semi-implicit synthetic fixture (not the fit's integral model)."""
    p = dict(zip(NAMES, [1.2, .8, .4, .3, .01, -.005, 180., 1.5]))
    p['backlash_half_gap'] = gap
    state = np.zeros(4)
    rows = []
    previous = float(plan[0]['time'])
    for row in plan:
        t = float(row['time'])
        effort = 120*(row['target']-state[0])-10*state[1]+row['excitation']
        dt = (t-previous)/20
        for _ in range(20):
            d = derivative(state, effort, p)
            state[1] += d[1]*dt
            state[3] += d[3]*dt
            state[0] += state[1]*dt
            state[2] += state[3]*dt
        rows.append(dict(time=t, segment=row['segment'], command=effort, effort=effort,
                         motor_position=float(state[0]), load_position=float(state[2])))
        previous = t
    return rows
