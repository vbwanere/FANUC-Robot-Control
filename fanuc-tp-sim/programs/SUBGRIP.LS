/PROG  SUBGRIP
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "AR[1] 1=close 2=open";
DEFAULT_GROUP	= 1,*,*,*,*;
/MN
   1:  !AR[1]: 1=close  2=open ;
   2:  IF AR[1]=2,JMP LBL[2] ;
   3:  DO[2:grip open]=OFF ;
   4:  DO[1:grip close]=ON ;
   5:  WAIT DI[1:grip closed]=ON TIMEOUT,LBL[9] ;
   6:  END ;
   7:  LBL[2] ;
   8:  DO[1:grip close]=OFF ;
   9:  DO[2:grip open]=ON ;
  10:  WAIT DI[2:grip open]=ON TIMEOUT,LBL[9] ;
  11:  END ;
  12:  LBL[9] ;
  13:  MESSAGE[GRIPPER TIMEOUT] ;
  14:  PAUSE ;
  15:  END ;
/POS
/END
