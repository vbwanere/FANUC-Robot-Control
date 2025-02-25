1: ! -------------------------------

2: ! Pick Program

3: ! AR[1]=Tray Number

4: ! -------------------------------

5:

6: UTOOL_NUM=1

7: UFRAME_NUM=0

8:

9: F[4:bPartMissingFlag]=(OFF)

10: ! Turn Vacuum Outputs OFF

11: DO[16:Close Grip Hs (Pulse)]=OFF

12: DO[17:Open Grip Hs (Pulse)]=PULSE,0.2sec

13:

14: ! Set UFrame and Current Nest Pos

15: IF AR[1]=1,JMP LBL[10]

16: IF AR[1]=2,JMP LBL[20]

17:

18: LBL[10]

19: R[19:nCurrUFrame]=2

20: ! Calculate Current Tray Nest Pos

21: CALL CALCTRAYNESTPOSN(4,R[10:nNest])

22: PR[12:pCurr]=PR[102:Scratch]

23: JMP LBL[30]

24:

25: LBL[20]

26: R[19:nCurrUFrame]=3

27: ! Calculate Current Tray Nest Pos

28: CALL CALCTRAYNESTPOSN(5,R[10:nNest])

29: PR[12:pCurr]=PR[102:Scratch]

30: JMP LBL[30]

31:

32: LBL[30]

33: CALL CHKPARTPRESFAULT

34:

35: ! Pick PCBA from Tray x

36: ! Setup Offsets

37: PR[101,1:Scratch]=0

38: PR[101,2:Scratch]=0

39: PR[101,3:Scratch]=R[30:nTCZoff]

40: OFFSET CONDITION PR[101:Scratch],UFRAME[R[19]]

41:L PR[10:pCurrNest] R[60:MaxSpd_mm\s]mm/sec CNT25 Offset

42:

43: ! Time Before(TB) used to

44: ! close grippers while moving

45:L PR[10:pCurrNest] R[62:PnPSpd_mm\s]mm/sec FINE TB R[50]sec,CALL CLSRGRIPTRAP

46: ! Ensure closed

47: CALL CLSGRIPTRAYPOSN

48:

49: ! Move to Tray Clear Position

50: ! Distance Before(DB) used to

51: ! check part presents moving

52:L PR[10:pCurrNest] R[126:MaxSpd_mm\s]mm/sec CNT50 Offset DB R[52]mm,CALL PICKHSSIGTRAP

53:

54: ! Move to Dial Nest 1 Clear

55: ! Setup Offsets

56: PR[101,1:Scratch]=0

57: PR[101,2:Scratch]=0

58: PR[101,3:Scratch]=R[31:nDCZoff]

59: OFFSET CONDITION PR[101:Scratch],UFRAME[1]

60:L PR[3:pDialNestRef] R[60:MaxSpd_mm\s]mm/sec CNT100 Offset

61:

62: ! If part missing move to good position

63: ! to open gripper

64: UTOOL_NUM=1

65: UFRAME_NUM=0

66: IF (F[4:bPartMissingFlag]=ON),JMP LBL[40]

67: CALL GETNEWCMD

68: IF R[1:nCmd]=11,JMP LBL[9999]

69:J PR[1:pHome] R[60:MaxSpd_mm\s]% CNT100

70: JMP LBL[9999]

71:

72: LBL[40]

73:L PR[6:pClrToOpn] R[60:MaxSpd_mm\s]mm/sec FINE

74: DO[16:Close Grip Hs (Pulse)]=OFF

75: DO[17:Open Grip Hs (Pulse)]=PULSE,0.2sec

76: WAIT .30(sec)

77:J PR[1:pHome] R[60:MaxSpd_mm\s]% CNT50

78: CALL SETCMDDONE

79: JMP LBL[9999]

80:

81: ! === Error Description Here==

82: LBL[9998]

83: R[3:nFaultNum]=XX

84: CALL SetFault

85: IF R[3:nFaultNum]=0,JMP LBL[9999]

86: JMP LBL[9998]

87:

88: ! === End ========================

89: LBL[9999]

90: F[4:bPartMissingFlag]=(OFF)