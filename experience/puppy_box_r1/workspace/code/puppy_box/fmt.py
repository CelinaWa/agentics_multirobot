import sys,json
for l in sys.stdin:
    try: d=json.loads(l)
    except: print(l.strip()); continue
    if d.get("step")=="perceive": d["paws"]=[(p["pix"],p["c"],p["n"],p.get("overhang")) for p in d["paws"]]
    print(json.dumps(d))
