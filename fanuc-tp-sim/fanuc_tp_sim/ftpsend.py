"""Push / pull .LS files to a real FANUC controller over FTP.

The controller's FTP server exposes its file devices as directories: MD: is
RAM (where selected programs live), MC: the memory card, FR: the FROM disk,
UD1: a USB stick. Uploading an .LS to MD: only works if the ASCII Upload
option is loaded; otherwise upload to FR:/UD1: and LOAD from the pendant, or
compile to .TP first with maketp.exe.

    python -m fanuc_tp_sim.ftpsend put 192.168.0.10 programs/MTEND.LS
    python -m fanuc_tp_sim.ftpsend get 192.168.0.10 MTEND.LS ./programs

Nothing here talks to the simulator -- it is the last step, after the program
runs clean in the sim.
"""
import ftplib
import os
import sys


def connect(host, user="", pw="", device="MD:"):
    f = ftplib.FTP(host, timeout=10)
    f.login(user, pw)
    if device:
        f.cwd(device)
    return f


def put(host, path, device="MD:", user="", pw=""):
    f = connect(host, user, pw, device)
    with open(path, "rb") as fh:
        f.storlines("STOR " + os.path.basename(path).upper(), fh)
    f.quit()
    return "uploaded %s to %s%s" % (path, host, device)


def get(host, name, dest=".", device="MD:", user="", pw=""):
    f = connect(host, user, pw, device)
    out = os.path.join(dest, name.upper())
    with open(out, "wb") as fh:
        f.retrlines("RETR " + name.upper(), lambda l: fh.write((l + "\n")
                                                               .encode()))
    f.quit()
    return "downloaded " + out


def ls(host, device="MD:", user="", pw=""):
    f = connect(host, user, pw, device)
    names = f.nlst()
    f.quit()
    return names


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a:
        print(__doc__)
    elif a[0] == "put":
        print(put(a[1], a[2], *(a[3:])))
    elif a[0] == "get":
        print(get(a[1], a[2], *(a[3:])))
    elif a[0] == "ls":
        print("\n".join(ls(a[1], *(a[2:]))))
