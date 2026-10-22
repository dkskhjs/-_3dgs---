# 찍방 공간 스캔 — 3DGS × 도면/BIM 오버레이 뷰어

3DGS 스캔(PLY · SPZ · SPLAT · KSPLAT)에 도면(PDF · DWG · DXF · 이미지)과 BIM/3D 모델(IFC · STL · OBJ · glTF · DAE · FBX)을 겹쳐 시공 품질과 공정을 확인하는 브라우저 앱입니다.

## 실행

빌드 과정은 없습니다. 정적 파일 서버로 `index.html`을 열면 됩니다 (라이브러리는 CDN에서 불러옵니다).

```bash
python3 -m http.server 8000
# http://localhost:8000
```

GitHub Pages: Settings → Pages → Branch `main` / root.

## 주요 기능

- 레이어(왼쪽): 3DGS 스캔 · 도면/BIM 업로드와 목록, 레이어별 투명도·색
- 특성(오른쪽): 평면/입면/단면 뷰, 스캔 색상(원본·흑백·BIM 겹침 강조), 앞에 표시(스캔/BIM)
- 스캔본 & 도면 위치 정렬: 바닥 인식, 도면 축척, 기준점·자동(벽 단면)·마커 정합, IFC 마커 정렬(최외곽 마커 기준 스케일), 미세 조정, 정렬 고정, Scan-to-BIM(IFC 생성)
- 자: 두 점 거리, 도면 끝점·IFC 객체 외곽선(꼭짓점·모서리) 스냅
- 이동: WASD · Shift, Space 바닥 착륙(눈높이 1.5 m 걷기)

## 라이브러리

three.js, Spark (3DGS), web-ifc, pdf.js, dxf-parser, libredwg-web (GPL-3.0).
