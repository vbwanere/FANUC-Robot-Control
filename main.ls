1: ! -------------------------------

2: ! Main Program

3: ! -------------------------------

4:

5: ! === Initialize =================

6: CALL INITIALIZE

7:

8: LBL[1:Loop]

9: ! === Get a PLC Command ==========

10: IF R[1:nCmd]=0,CALL GETNEWCMD

11:

12:

13:

14: ! -------------------------------

15: ! Check for pick tray commands

16: IF R[1:nCmd]>=11000 AND R[1:nCmd]<12000,JMP LBL[1100]

17: IF R[1:nCmd]>=12000 AND R[1:nCmd]<13000,JMP LBL[1200]

18: ! -------------------------------

19: ! Check for other commands

20: SELECT R[1:nCmd]=1,CALL MHOME

21:   =2,CALL MSERVICE

22:   =11,CALL MPLACEDIAL

23: ELSE,JMP LBL[9998]

24: JMP LBL[9000]

25: ! -------------------------------

26: LBL[1100:Tray1]

27: CALL MPICKTRAY(1)

28: JMP LBL[9000]

29: ! -------------------------------

30: LBL[1200:Tray2]

31: CALL MPICKTRAY(2)

32: JMP LBL[9000]

33: ! -------------------------------

34: LBL[9000:Command Complete]

35: TIMER[1]=STOP

36: JMP LBL[1]

37:

38: ! ============================

39: ! Invalid Command Number

40: LBL[9998:Fault]

41: TIMER[1]=STOP

42: R[1:nCmd]=0

43: R[3:nFaultNum]=11

44: CALL SETFAULT

45: IF R[3:nFaultNum]=0,JMP LBL[1]

46: JMP LBL[9998]

47:

48: ! === End ========================

