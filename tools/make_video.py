"""Post-process the AVI written by demo_recorder into the submission MP4 (1280x720, H.264).

Adds a Japanese title card, per-phase captions and an ending card.
The caption timings (caps) match the submitted run; adjust them for a new recording.
Requires: opencv-python, Pillow, ffmpeg, a Japanese font (Noto Sans CJK).
    python3 tools/make_video.py ~/pai_demo.avi pai_demo_final.mp4
Font path can be changed with the environment variable JP_FONT_DIR.
"""
import os
import sys, subprocess, cv2, numpy as np
from PIL import Image, ImageDraw, ImageFont
SRC, OUT = sys.argv[1], sys.argv[2]
FD=os.environ.get('JP_FONT_DIR','/usr/share/fonts/opentype/noto')
FONT=os.path.join(FD,'NotoSansCJK-Bold.ttc'); FONTR=os.path.join(FD,'NotoSansCJK-Regular.ttc')
f=lambda s,b=True: ImageFont.truetype(FONT if b else FONTR, s, index=0)
W,H,FPS=1280,720,10
T0,T1=4.0,92.0
caps=[(4,21.5,'① 目標「停止標識（stop sign）」を探索 → YOLOで発見して接近'),
      (21.5,25.5,'① 停止標識に到着 → 2秒停止して後退'),
      (25.5,44,'② 目標「人（person）」を探索・接近（停止標識は目標外なので無視）'),
      (44,47.5,'② 人の手前で停止'),
      (47.5,82.5,'③ 目標「消火栓（fire hydrant）」を探索・接近'),
      (82.5,86,'③ 消火栓に到着'),
      (86,999,'ミッション完了：3つの目標物すべてに順番に到達')]
def card(lines, sub=None):
    im=Image.new('RGB',(W,H),(18,24,38)); d=ImageDraw.Draw(im); y=200
    for txt,size,col in lines:
        w=d.textlength(txt,font=f(size)); d.text(((W-w)/2,y),txt,font=f(size),fill=col); y+=size+30
    return cv2.cvtColor(np.array(im),cv2.COLOR_RGB2BGR)
ff=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','bgr24','-s',f'{W}x{H}','-r',str(FPS),'-i','-',
  '-vf','fps=30','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',OUT],stdin=subprocess.PIPE)
title=card([('Physical AI 最終課題',40,(150,200,255)),
            ('YOLOで目標物を「探して・近づく」差動二輪ロボット',46,(255,255,255)),
            ('ROS 2 Humble ＋ Gazebo ／ 学習済みYOLOv8n（推論のみ）',30,(200,200,200)),
            ('タスク：停止標識 → 人 → 消火栓 の順に自律で探索・到達',30,(255,210,120)),
            ('動画は等倍速（1倍速）です',28,(180,180,180))])
for _ in range(int(6*FPS)): ff.stdin.write(title.tobytes())
cap=cv2.VideoCapture(SRC); i=0
while True:
    ok,fr=cap.read()
    if not ok: break
    t=i/FPS; i+=1
    if t<T0: continue
    if t>T1: break
    canvas=np.zeros((H,W,3),np.uint8); canvas[85:85+550]=cv2.resize(fr,(1280,550))
    im=Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB)); d=ImageDraw.Draw(im)
    d.rectangle([0,0,W,85],fill=(18,24,38))
    cap_txt=next(c for a,b,c in caps if a<=t<b)
    d.text((24,20),cap_txt,font=f(34),fill=(255,255,255))
    d.text((24,85+550+8),'左：俯瞰カメラ（オレンジ色の小さな車体がロボット）　右：ロボット前方カメラ＋YOLO推論結果（赤枠＝現在の目標）',font=f(20,False),fill=(220,220,220))
    d.text((24,85+550+42),'AIの使い方：前方カメラ画像をCOCO学習済みYOLOv8nで物体検出し、目標物の位置と大きさから速度指令（/cmd_vel）を計算',font=f(20,False),fill=(170,200,255))
    ff.stdin.write(cv2.cvtColor(np.array(im),cv2.COLOR_RGB2BGR).tobytes())
end=card([('結果',40,(150,200,255)),
          ('停止標識・人・消火栓の3つすべてに自律で到達（約80秒）',40,(255,255,255)),
          ('探索（旋回）→ 接近（中心合わせのP制御）→ 到着判定（検出枠の大きさ）',28,(220,220,220)),
          ('目標外の物体（人・箱・円柱など）は無視して指定順に巡回',28,(220,220,220)),
          ('応用：施設巡回・見守り・点検ロボットの「指定物を探して確認に行く」動作',28,(255,210,120))])
for _ in range(int(7*FPS)): ff.stdin.write(end.tobytes())
ff.stdin.close(); ff.wait()
