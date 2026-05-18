## 

## scan
1. `[scan] exec_scan ENTERED` → scan 함수 진입 확인
2. `[scan/ALIGN]...` → ALIGN PID 제어가 정상적으로 동작하는지
3. `[scan] ALIGN done (yaw≈0°)` → ALIGN 종료 확인
4. `[scan/SWEEP] ...` → 스윕 왕복 및 yaw 제어 확인
5. `[scan] vision objects=...` → Vision 입력 정상 수신 여부
6. `[scan] report_on_found ...` → 발견 동작 확인