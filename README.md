🟢 v1 (done)
Polling-based ADC + UART print

🟢 v2 (done)
Timer interrupt–based sampling

- Set a flag in the ISR
- Read ADC in the main loop
- Store sampled values in RAM

🟢 v3 (done)
Add ring buffer

- struct Sample { timestamp, temp1, temp2 }
- Implement circular buffer
- Handle buffer full condition (overwrite or drop)

🔵 v4 (next)
UART CLI

- Receive string input
- Parse commands
- Print system status
- Example commands: `status`, `start`, `stop`

🔴 v5
Reliability layer

- Watchdog timer
- Error counters
- Safe behavior on faults