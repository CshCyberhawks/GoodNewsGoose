import os
try:
    import httplib  # python < 3.0
except:
    import http.client as httplib


def have_internet() -> bool:
    conn = httplib.HTTPSConnection("8.8.8.8", timeout=5)
    try:
        conn.request("HEAD", "/")
        return True
    except Exception:
        return False
    finally:
        conn.close()

if have_internet():
    os.system("git submodule update --remote")
