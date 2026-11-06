"""Print RUNS.md rows from output/stance_batch.jsonl for the given tags: python code/puppy_box/batch_rows.py s8 s9 ..."""
import sys, json
want = set(sys.argv[1:])
for l in open('output/stance_batch.jsonl'):
    d = json.loads(l)
    if want and d['tag'] not in want:
        continue
    parts = []
    for t in d['tries']:
        w = t.get('widths') or []
        c = t.get('carry') or {}
        parts.append('%s; paw %s; close %s; widths %s%s' % (t['why'][:70], [round(v, 3) for v in (t.get('paw') or [])][:2],
                     t.get('close') and round(t['close'], 4), ' '.join('%.4f' % v for v in w[:12]),
                     ('; carry walked %s dropped %s' % (c.get('walked_total_m'), c.get('dropped'))) if c else ''))
    op = (d['tries'][-1].get('operator') or '')[:110]
    res = 'INSIDE' if d['inside'] else ('NOT inside' if d['inside'] is False else 'no release')
    print('| %s (batch, kw %s) | operator paw %s | %s | %s | %s |' % (d['tag'], d.get('kw', {}), d['op_paw'], ' // '.join(parts), res, op))
