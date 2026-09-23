import serial
import time

# Configureaza portul COM unde este conectat Arduino (schimba 'COM3' cu portul tau)
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

def run_hil_tests():
    print("--- Starting Hardware-in-the-Loop (HiL) Validation ---")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2) # Asteptam resetarea placii
    except Exception as e:
        print(f"Eroare de conectare: {e}")
        return

    test_samples = 0
    passed_tests = 0

    while test_samples < 20: # Rulam 20 de cicluri de testare
        line = ser.readline().decode('utf-8').strip()
        
        if not line:
            continue
            
        data = line.split(',')
        if len(data) != 7:
            continue # Ignoram liniile incomplete
            
        timestamp, bpm, temp, piezo, gsr, score, state = map(float, data)
        test_samples += 1
        
        print(f"\n[Sample {test_samples}] Telemetry recv: {line}")
        
        # --- ASERȚIUNI DE TESTARE (VALIDĂRI AUTO) ---
        
        # Test 1: Validare limite BPM (Senzorul MAX30102)
        if 0 <= bpm <= 200:
            print("  [PASS] BPM in limite fiziologice sigure.")
            passed_tests += 1
        else:
            print(f"  [FAIL] BPM anomalic: {bpm}")
            
        # Test 2: Validare Temp (Senzorul DS18B20 pe OneWire)
        if 15.0 <= temp <= 45.0 or temp == 0.0:
            print("  [PASS] Senzorul de temperatura raspunde corect.")
            passed_tests += 1
        else:
            print(f"  [FAIL] Eroare senzor DS18B20 (Temp: {temp})")
            
        # Test 3: Logica de sistem (FSM state machine)
        if state in [0, 1, 2]:
            print("  [PASS] FSM State Machine valid.")
            passed_tests += 1
        else:
            print(f"  [FAIL] Stare necunoscuta sistem: {state}")

    ser.close()
    
    # Raport Final
    total_assertions = test_samples * 3
    print("\n=== RAPORT FINAL HIL ===")
    print(f"Teste rulate: {total_assertions}")
    print(f"Teste trecute: {passed_tests} ({(passed_tests/total_assertions)*100:.1f}%)")

if __name__ == '__main__':
    run_hil_tests()