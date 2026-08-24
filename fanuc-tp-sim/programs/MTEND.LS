/PROG  MTEND
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "lathe tend";
DEFAULT_GROUP	= 1,*,*,*,*;
/MN
   1:  !lathe machine tending ;
   2:  UFRAME_NUM=0 ;
   3:  UTOOL_NUM=1 ;
   4:  R[1:parts]=0 ;
   5:  CALL SUBGRIP(2) ;
   6:  DO[6:door open]=OFF ;
   7:  DO[7:door close]=ON ;
   8:J P[1:home] 50% FINE ;
   9:  LBL[1] ;
  10:  IF DI[10:part at infeed]=OFF,JMP LBL[99] ;
  11:  !--- pick from infeed --- ;
  12:J P[2:inf apr] 50% CNT50 ;
  13:L P[3:inf pick] 200mm/sec FINE ;
  14:  CALL SUBGRIP(1) ;
  15:  IF DI[9:part in grip]=OFF,JMP LBL[98] ;
  16:L P[2:inf apr] 200mm/sec CNT50 ;
  17:  !--- open door, load chuck --- ;
  18:  DO[7:door close]=OFF ;
  19:  DO[6:door open]=ON ;
  20:  WAIT DI[5:door open]=ON TIMEOUT,LBL[97] ;
  21:J P[4:door apr] 50% CNT50 ;
  22:L P[5:chk apr] 300mm/sec CNT10 ;
  23:L P[6:chuck] 100mm/sec FINE ;
  24:  DO[4:chuck open]=OFF ;
  25:  DO[3:chuck close]=ON ;
  26:  WAIT DI[3:chuck closed]=ON TIMEOUT,LBL[97] ;
  27:  CALL SUBGRIP(2) ;
  28:L P[5:chk apr] 200mm/sec CNT50 ;
  29:J P[4:door apr] 50% CNT50 ;
  30:  DO[6:door open]=OFF ;
  31:  DO[7:door close]=ON ;
  32:  WAIT DI[6:door closed]=ON TIMEOUT,LBL[97] ;
  33:  !--- run the machining cycle --- ;
  34:  TIMER[1]=RESET ;
  35:  TIMER[1]=START ;
  36:  DO[5:cycle start]=ON ;
  37:  WAIT DI[8:cycle done]=ON TIMEOUT,LBL[97] ;
  38:  DO[5:cycle start]=OFF ;
  39:  TIMER[1]=STOP ;
  40:  R[2:cyc sec]=TIMER[1] ;
  41:  !--- unload --- ;
  42:  DO[7:door close]=OFF ;
  43:  DO[6:door open]=ON ;
  44:  WAIT DI[5:door open]=ON TIMEOUT,LBL[97] ;
  45:J P[4:door apr] 50% CNT50 ;
  46:L P[5:chk apr] 300mm/sec CNT10 ;
  47:L P[6:chuck] 100mm/sec FINE ;
  48:  CALL SUBGRIP(1) ;
  49:  DO[3:chuck close]=OFF ;
  50:  DO[4:chuck open]=ON ;
  51:  WAIT DI[4:chuck open]=ON TIMEOUT,LBL[97] ;
  52:L P[5:chk apr] 200mm/sec CNT50 ;
  53:J P[4:door apr] 50% CNT50 ;
  54:  DO[6:door open]=OFF ;
  55:  DO[7:door close]=ON ;
  56:  !--- drop at outfeed --- ;
  57:J P[7:out apr] 50% CNT50 ;
  58:L P[8:out drop] 200mm/sec FINE ;
  59:  CALL SUBGRIP(2) ;
  60:L P[7:out apr] 200mm/sec CNT50 ;
  61:  R[1:parts]=R[1:parts]+1 ;
  62:J P[1:home] 50% FINE ;
  63:  JMP LBL[1] ;
  64:  LBL[97] ;
  65:  MESSAGE[CELL TIMEOUT - CHECK IO] ;
  66:  PAUSE ;
  67:  JMP LBL[1] ;
  68:  LBL[98] ;
  69:  MESSAGE[NO PART IN GRIPPER] ;
  70:  PAUSE ;
  71:  JMP LBL[1] ;
  72:  LBL[99] ;
  73:  MESSAGE[INFEED EMPTY] ;
  74:  END ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,
	J1=      0.000 deg,
	J2=      0.000 deg,
	J3=      0.000 deg,
	J4=      0.000 deg,
	J5=    -30.000 deg,
	J6=      0.000 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    650.000  mm,	Y =   -550.000  mm,	Z =    500.000  mm,
	W =    180.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
P[3]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    650.000  mm,	Y =   -550.000  mm,	Z =    300.000  mm,
	W =    180.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
P[4]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    900.000  mm,	Y =    100.000  mm,	Z =    700.000  mm,
	W =    -90.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
P[5]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    900.000  mm,	Y =    300.000  mm,	Z =    700.000  mm,
	W =    -90.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
P[6]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    900.000  mm,	Y =    500.000  mm,	Z =    700.000  mm,
	W =    -90.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
P[7]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    650.000  mm,	Y =   -800.000  mm,	Z =    500.000  mm,
	W =    180.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
P[8]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =    650.000  mm,	Y =   -800.000  mm,	Z =    300.000  mm,
	W =    180.000 deg,	P =      0.000 deg,	R =      0.000 deg
};
/END
