import walkplan as w, math, itertools, sys
start = tuple(float(v) for v in sys.argv[1:4]); start = (start[0], start[1], math.radians(start[2]))
tgt = (0.333, 0.359, 0.0)
acts = [('turn', c) for c in (-100, -90, -80, -60, -45, -30, -15, 15, 30, 45, 60, 80, 90, 100)] + [('walk', c) for c in (-0.1, 0.1, 0.2)]
best = []
for n in (2, 3, 4):
    for seq in itertools.product(acts, repeat=n):
        q, ok = start, True
        for i, a in enumerate(seq):
            q = w.step(q, a)
            if 0.75 - q[0] < 0.3:
                ok = False; break
        if not ok: continue
        e = math.hypot(q[0]-tgt[0], q[1]-tgt[1]) + 0.25*abs(q[2]) + 0.01*n
        best.append((e, seq, q))
best.sort(key=lambda t: t[0])
for e, s, q in best[:6]:
    print(round(e,3), s, [round(q[0],3), round(q[1],3), round(math.degrees(q[2]),1)])
