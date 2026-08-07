from pathlib import Path
import py_compile

root = Path(__file__).resolve().parents[1] / 'uncc_example'
errors = []

for path in root.glob('*.py'):
    try:
        py_compile.compile(str(path), doraise=True)
        print(f'OK  {path.name}')
    except Exception as exc:
        errors.append((path, exc))
        print(f'ERR {path.name}: {exc}')

if errors:
    raise SystemExit(1)
