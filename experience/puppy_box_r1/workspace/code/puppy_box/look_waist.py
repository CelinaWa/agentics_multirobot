"""CLI: look at a paw hidden below the view with the waist yawed (function lives in lowtilt_v4.look_waist).
python code/puppy_box/look_waist.py [waist_yaw=-0.35]"""
import sys, json, importlib.util
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('s', 'code/puppy_box/lowtilt_v4.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
if __name__ == '__main__':
    print(json.dumps(m.look_waist(Context(), float(sys.argv[1]) if len(sys.argv) > 1 else -0.35, log=lambda *a: None), default=str)[:1500])
