```mermaid
graph TB
  START(START) --> start_wait(WAIT:millsec=500)
  start_wait --> SETPOSE.1(SETPOSE:x=-0.8,y=2.0,z=90.0)
  SETPOSE.1 --> END(END)
```
