"""Double-click launcher: mode 4, 1000 OHTs, probability 10%, 8 seconds."""
import os
import sys
from pathlib import Path
root=Path(__file__).resolve().parent
os.chdir(root)
(root/'runs').mkdir(exist_ok=True)
with (root/'runs/gui_console.log').open('w',encoding='utf-8',buffering=1) as log:
    sys.stdout=sys.stderr=log
    sys.path.insert(0,str(root))
    from view_hoist import main
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        import ctypes
        ctypes.windll.user32.MessageBoxW(0,str(root/'runs/gui_console.log'),'실행 오류: 로그를 확인하세요',16)
