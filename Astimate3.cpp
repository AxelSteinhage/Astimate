//
// Astimate 3 
// 2021 by Axel.Steinhage@web.de
//
// Conventions:
// - self-defined types start with upper case letter
// - bitmap-variables' names are all in capital
// - byte-variables are all in lower case
// - other variable types start with upper case letters
//
// Information:
//  material signature: white Q=001,R=002,B=004,N=008,P=010  black Q=020,R=040,B=080,N=100,P=200
//  x&(x-1): clear lowest 1 bit, x&(-x): isolate lowest 1 bit
//  x^(x-1): isolate lowest 1 bit and flip lower zeros to 1
//
//
// Hash tables structure
//
// Transposition table: 16 bytes per entry, 2 entries per slot
// 0: depth, 1: flags, 2-3:value, 4-5:move, 6-7: nn,  8-15: lock, 
// 				flags: (0: upper, 1: lower, 2: first entry, 3: from previous game, ...)
//
// Pawn hash table: 40 bytes per entry
// Byte 0-7:lock, 8-13: white pawn attacks, 14-15: opening value,
// 16-17: endgame value, 18-23: black pawn attacks 
// 24: flags (0:vertical- 1:horizontal opposition,..), 25-30: passers, 31: nn, 32:nn,
// 33-38: weak pawns, 39: nn  
//
// Material hash table: 8 bytes per entry
// Byte 0-1:material value, 2-7:lock    
//
// Evaluation hash table: 8 Bytes per entry
// Byte 0-1:evaluation, 2-7:lock 
//
 
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

#define MaxScore 	30256												// maximum material score
#define PVN			0xFFFF												// size of PV table

typedef unsigned char 		Byte;	
typedef unsigned long long 	BitMap;
typedef unsigned long 		Fbyte;
typedef unsigned short 		Dbyte;										

#include "Astimate.h"													// load constants

typedef struct															// game structure
{
	struct	{Byte type,index;} 				Piece[2][64];				// [color][square]
	struct	{Byte type,square;} 			Officer[2][16];				// [color][index]
	struct	{Byte officers,pawns;} 			Count[2];					// piece count
	struct  {short Open,End;} 				Psv[2];						// piece square values
	struct	{Byte from,to,cap,castles,ep,fifty,prom; Dbyte Mov;			// origin, destination, captured piece,castles, ep square, fifty move counter ...
			 bool check; BitMap	HASH;} 		Moves[1000];				// ... promoted piece, compressed move, check, HASH value, move list		  									
	Dbyte									Killer[256][3][2];			// killer moves
	Dbyte									Counter[2][17][64];			// counter move table
	BitMap 									POSITION[2][7];				// positions of pieces [color][all, king, ..., pawns]
	BitMap 									ROTATED[4];					// bitmaps of pieces [all, ]
	BitMap  								PHASH;						// pawn hash
	BitMap									MHASH;						// material hash
	BitMap									NODES;						// node counter	
	
	BitMap									TREESIZE[250];
	Dbyte									Rootmvs[250];
	
	Dbyte									Currm;						// move currently calculated
	Dbyte									Bestmove;					// Bestmove at root position
	Dbyte									Lastbest;					// best move of previous iterartion
	Dbyte									Move2Make;					// current best root move
	Dbyte									Pmove;						// ponder move
	Dbyte									Move_n;						// move number
	Dbyte									Move_m;						// moves made in initial position
	Dbyte									Move_r;						// move number at root
	Dbyte									Matsig;						// material signature
	short									Val;						// value of root position
	short									Alpha;						// lower bound at root
	short									Beta;						// upper bound at root
	short									Lastval;					// last best value
	Byte									idepth;						// depth of iteration
	Byte									phase;						// game phase
	Byte									ngen;						// number of generated moves
	Byte									perm;						// permanent check counter
	Byte									noloose;					// non loosing moves
	Byte									color;						// color to move
	Byte									Threadn;					// number of thread
	pthread_t								Tid;						// thread ID
	bool volatile							Finished;					// calculation is finished
} Game;

typedef struct
{
	BitMap									AMVS;						// all moves (for single move detection)
	BitMap									ADES;						// all destinations stm officers
	BitMap									OFFM[16];					// stm officers' moves
	BitMap									PAWM[4];					// stm pawns' moves (capleft,single,capright,double)
	Dbyte									Prop[8];					// stm promotion piece information
	BitMap									OATK[6];					// oponent's attacks (all, k-n)
	BitMap									OCHK[4];					// oponent's checks (rook/queen, bishop/queen, knight, pawn)
	BitMap									DCHK;						// discovered check pieces stm	
	BitMap									PINS;						// pinned stm pieces
	BitMap									PINO;						// pinned oponent's pieces
	BitMap									CMB;						// current move bitmap
	Byte									s;							// current pickmove stage
	Byte									o;							// current pickmove officer
	Byte									flg;						// pickmove flags (0: positive SEE, 2: save square, 7: counter moves)
	Byte									cp;							// count pieces that can move
	Dbyte									Bestmove;					// current Bestmove						
} Mvs;

struct Opt {															// Astimate's options
	char	Name[20];													// name of option
	bool 	Val,Change;}												// default value and change
	
Options[8] = {
 	{"UCI",						true,	false},							// uci or terminal mode 	
	{"UseOpeningTree", 			true,	false},							// use opening book tree
 	{"UseOpeningPoslib",		false,	false},							// use opening book positions
 	{"DebugMode",				true,	false},							// debug mode
 	{"InteriorNodeRecog",		true,  	false},							// interior node recognition
 	{"PrePromExtension",		true,	false},							// extend pre promotion moves
 	{"RecaptureExtension",		true,	false},							// extend recaptures
 	{"AdaptNull",				false,  false}							// decrement null reduction when king is in danger
};

struct Par {															// Astimate's parameters
	char	Name[30];													// name of the parameter
	short	Val,Low,High,Change;}										// default, lower, upper value and change	

Paras[96]	= {														
	{"Hash",					256, 	1,  4096,  0},					// transposition table size (MB)
	{"PawnTable",				 10, 	1,   100,  0},					// pawn hash table size (MB)
	{"MaterialTable",			  1, 	1,     5,  0},					// material hash table size (MB)
	{"EvaluationTable",			  2, 	1,     5,  0},					// evaluation hash table size (MB)
	{"KingValue",			  10000,10000, 10000,  0},					// value of king	
	{"QueenValue",				975,  500,  1500, 10},					// value of queen
	{"RookValue",				500,  200,  1000, 10},					// value of rook
	{"BishopValue",				325,  100,   800, 10},					// value of bishop
	{"KnightValue",				325,  100,   800, 10},					// value of knight
	{"PawnValue",				100,    0,   300, 10},					// value of pawn
	
	{"PairValue",				 50,    0,   200, 10},					// value of bishop pair
	{"PST-Influence",			 10,    0,   100,  5},					// influence of pst table
	{"SideToMoveBonus",			  5,    0,   100,  5},					// side to move bonus
	{"GeneralPawnKingDist",		  1,	0,	  10,  1},					// general pawn-king distance
	{"PawnChains",				  7,	0,	  20,  1},					// pawn chains
	{"PawnPhallanx",			  5,	0,	  20,  1},					// pawn phallanx
	{"DoublePawnOpening",		 20,	0,	 100,  5},					// double pawn opening
	{"DoublePawnEndgame",		  0,	0,	 100,  5},					// double pawn endgame
	{"IsolatedPawnOpening",		 10,	0,	 100,  5},					// isolated pawn opening
	{"IsolatedPawnEndgame",		  0,	0,	 100,  5},					// isolated pawn endgame
	
	{"BackwardPawnOpening",		 10,	0,	 100,  5},					// backward pawn opening
	{"BackwardPawnEndgame",		  5,	0,	 100,  5},					// backward pawn endgame
	{"WeakPawnOpening",		 	 10,	0,	 100,  5},					// weak pawn opening
	{"WeakPawnEndgame",		  	  0,	0,	 100,  5},					// weak pawn endgame
	{"PassedPawnKingDist",		  2,    0,    20,  1},					// passed pawn king distance
	{"PassedPawnOpening",		  1,	0,	  20,  1},					// passed pawn general opening bonus
	{"PassedPawnEndgame",		  4,	0,	  20,  1},					// passed pawn general endgame bonus	
	{"ConnectedPassers(inv.)",	  4,    1,	  20,  1},					// connected passed pawns (inverse)
	{"CandidatePawnKingDist",	  1,    0,    20,  1},					// distance candidate pawn to king
	{"CandidatePawnOpening",	  1,    0,	  20,  1},					// general candidate pawn opening bonus
	
	{"CandidatePawnEndgame",	  2,    0,	  20,  1},					// general candidate pawn endgame bonus
	{"PawnShield",				  7,	0,	  20,  1},					// pawn shield bonus per pwan
	{"PassedPawnTelestop",		  5,	0,	  20,  1},					// passed pawn telestop
	{"PassedPawnStop",			  5,	0,	  20,  1},					// passed stop
	{"PasserRaceBonus",			125,	0,	 500, 10},					// race bonus 
	{"PasserFreePath",			 50,	0,	 300, 10},					// free path for passed pawn outside Berger
	{"PasserStopAttacked",		  5,	0,	  20,  1},					// stop square attacked
	{"BishopControlsStop",		  5, 	0,	  20,  1},					// bishop controls stop square
	{"VerticalOpposition",		 10,	0,	 100,  5},					// vertical opposition
	{"HorizontalOpposition",	  5,	0,	 100,  5},					// horizontal opposition
	
	{"LazyEval",				  0,	0,	 500, 10},					// lazy eval limit
	{"PinnedPieces",			  5,    0,    20,  1},					// malus for pinned pieces
	{"DiscoveredCheck",		  	  0,    0,    20,  1},					// discovered check pieces
	{"OfficersAttacked",		  5,	0,	  50,  1},					// count attacks on officers
	{"KingAttacked",			  5,	0,	  50,  1},					// count attacks on king
	{"WeakPawnAttacked",		  5,	0,	  50,  1},					// count attacks on weak pawns
	{"QueenMobility",			  2,	0,	  20,  1},					// count queen moves
	{"TrappedQueen",			 20,	0,   200,  5},					// malus for trapped queen
	{"RookMobility",			  2,	0,	  20,  1},					// count rook moves
	{"TrappedRook",			 	 10,	0,   200,  5},					// malus for trapped rook
	
	{"BishopMobility",			  2,	0,	  20,  1},					// count bishop moves
	{"TrappedBishop",			 10,	0,   200,  5},					// malus for trapped bishop
	{"KnightMobility",			  2,	0,	  20,  1},					// count knight moves
	{"TrappedKnight",			 10,	0,   200,  5},					// malus for trapped knight
	{"CenterControl",			  2,	0,	  20,  1},					// count uncontrolled center squares
	{"PawnsKingAttack",			  5,	0,	  20,  1},					// pawns attack king 1 area	
	{"RookHalfopenOpening",		  5,	0,	  20,  1},					// rook on half open file opening
	{"RookHalfopenEndgame",		  5,	0,	  20,  1},					// rook on half open file endgame
	{"RookOpenOpening",			  5,	0,    20,  1},					// rook on open file opening
	{"RookOpenEndgame",			  5,	0,    20,  1},					// rook on open file endgame
	
	{"DoubleRooks",			  	  5,	0,    50,  1},					// double rooks
	{"RookBehindPasser",		  5,	0,	  50,  1},					// rook behind passer
	{"RookBehindOpPasser",		 10,	0,	  50,  1},					// rook behind opponent passer
	{"RookOn7th",				 10,	0,	  50,  1},					// rook on 7th rank
	{"RookOn7thKingOn8th",		  5,	0,	  50,  1},					// rook on 7th and king on 8th
	{"PawnsOnBishopColor",		  0,	0,	  10,  1},					// pawns on bishop color
	{"PawnDefendsBishop",		  0,	0,	  10,  1},					// bishop defended by pawwn
	{"BishopOutpost",			 10,	0,	  50,  1},					// bishop outpost
	{"PawnDefendsKnight",		  0,	0,	  10,  1},					// knight defended by pawwn
	{"KnightOutpost",			 15,	0,	  50,  1},					// knight outpost
	
	{"CastlesPossible",		  	  5,	0,	  50,  1},					// castles possible
	{"CastlesDone",		     	  5,	0,	  50,  1},					// castles done
	{"BishopOnh7Opening",	    150,	0,	 300, 10},					// bishop trapped on h7 opening
	{"BishopOnh7Endgame",		 50,	0,	 300, 10},					// bishop trapped on h7 endgame
	{"DrawScore",				  0,    0,  1000, 10},					// draw score. 500=0
	{"Depth1Pruning",			235,	0,	1200, 10},					// pruning at depth 1
	{"Depth2Pruning",			450,	0,  1200, 10},					// pruning at depth 2
	{"Depth3Pruning",			625,	0,  1200, 10},					// pruning at depth 3
	{"Depth4Pruning",		   1125,	0,	1200, 10},					// pruning at depth 4
	{"Q-PruningMiddlegame",		 80,	0,	 500, 10},					// quiescence pruning middlegame
	
	{"Q-PruningEndgame",		180,	0,   500, 10},					// quiescence pruning endgame
	{"ChecksInQsearch",			  1,	0,	  10,  1},					// check in quiescence search
	{"NullmoveReduction",		  4,	0,	   6,  1},					// nullmove reduction 4
	{"MulticutReduction",		  3,	0,	   6,  1},					// multicut reduction 3
	{"IIDReduction",			  4,	0,	   6,  1},					// internal iterative deepening reduction 4
	{"DeepsearchThreshold",		150,    0,  1000, 10},					// nullmove deepsearch threshold
	{"MulticutCount",			  6,	4,   100,  1},					// number of multicut moves to check
	{"MulticutSuccess",			  3,	1,	  10,  1},					// number of successful multi cuts
	{"LMRReduction",			  6,    0,    10,  1},					// LMR reduction
	{"AspirationMargin",		 50,    0,	 200, 10},					// size of aspiration window
	
	{"SuddenDeathMoves",		 30,	1,	 200,  1},					// estimated number of moves to go
	{"SuddenDeathWInc",			  7,	1,	 200,  1},					// sudden death moves to go with increment
	{"HardBreakTimeFactor",		  5,	1,	  20,  1},					// stop calculation after x times planned time
	{"HelperThreads",			  4, 	0,	  50,  0},					// number of helper threads
	{"LMRBadMoveReduction",		  2,    0,     3,  0},					// reduction for bad moves
	{"PromotionPruning",		800,  500,	1500,  0}					// promotion pruning value in Qsearch
};

short 	Maxply,(*recog[1024])(Game*,Byte*),Malpha,Mbeta,Mval;			// recognition function pointers
Byte 	*hash_t,*phash_t,*mhash_t,*ehash_t,level,maxdepth,nmate,idepth;	// transposition-,pawn-,material and evaluation hash table
BitMap	HEN,PEN,MEN,EEN,DEN,HASHFILL,ALLNODES,MAXNODES;             	// number of transposition table entries
Dbyte	Pv[PVN],Movestogo;									            // maximal ply, pv table	
clock_t StartTime;														// start time of calculation
Fbyte	Tmax,Wtime,Btime,Winc,Binc,Movetime;							// time variables										
volatile bool Stop,Ponder;												// stop calculation, ponder mode

Dbyte	Line[]={0x0900,0x363F,0x1109,0x2E36,0x1A11,0x2D2E,0x221A,0x252D,0x2B22,0x2D25,0x8088,0x352D,0x3600,0x3635};

void	SetGlobalDefaults();											// set global variables to initial defaults
void 	InitDataStructures();											// initialize basic data structures
void 	InitNewGame(Game*);												// initialize a new game
void 	ClearTables();													// clear all hashtables
void 	*ScanInput(void*);												// scan ascii input
void 	ParseFen(Game*,char*);											// scans fen string and sets all variables
void	GetPosition(Game*,char*);										// reads a psosition from input
bool 	TestCheck(Game*,Byte);											// test for check on square k
bool 	TestMCheck(Game*,Mvs*,Dbyte);									// move may announce check?
bool 	TestAttk(Game*,Byte,Byte,Byte);									// does piece attack higher piece on square?	
void 	GenMoves(Game*, Mvs*);											// generate moves
Dbyte 	CodeMove(Game*,char*);											// get move as number
void 	UncodeMove(Dbyte,char*);										// get move as string
bool	TestLine(Game*,Byte);											// test sequence of moves
void 	Move(Game*,Dbyte);												// make a move
void 	UnMove(Game*);													// take back move
Dbyte 	PickMove(Game*,Mvs*);											// pick moves from movelist
void 	Perft(Game*,Byte);												// get Perft(d) of position				
void 	*PerftCount(void*);												// recursive node counting
void 	Divide(Game*,Byte);												// print divide of position
void 	StoreHashP(Game*,BitMap);										// store entry in perft table
BitMap 	GetHashP(Game*);												// get entry from perft table
BitMap 	GetHash(Game*);													// get entry from transposition table
void	StoreHash(Game*,short,Dbyte,Byte,Byte);							// store transposition information
void 	PrintBM(BitMap);												// print bitmap
short	Popcount(BitMap);												// count set bits in bitmap
Dbyte	Book(Game*);													// check opening books
bool 	DrawTest(Game*);												// position is draw?	
bool 	SEE(Game*,Dbyte,short);											// SEE of current move larger than threshold
void	PrintMoves(Game*);												// print all legal moves
void	PrintPosition(Game*);											// print board
short 	MatEval(Game*);													// material evaluation
short 	Evaluation(Game*,Mvs*,short,short);								// evaluation
short	Qsearch(Game*,short,short,Byte);								// quiescence search
short	Search(Game*,short,short,Byte,Dbyte*);							// recursive negamax search
void	*SmpSearch(void*);												// smp main seach thread
void	IterateSearch(Game*);											// find a best move
void 	GetPV(Game*);													// retrieve principal variation
void 	PrintPV(Game*,short,short,short);								// print PV
void 	PrintCurrent(Game*,Dbyte,Dbyte);								// prints current move

short 	NoRecog(Game*,Byte*);											// no recognizer found
short 	KvK(Game*,Byte*);												// KvK endgame recognition function
short 	KPvK(Game*,Byte*);
short 	KvKP(Game*,Byte*);
short 	KNvK(Game*,Byte*);
short 	KvKN(Game*,Byte*);
short 	KBvK(Game*,Byte*);
short 	KvKB(Game*,Byte*);
short 	KRvK(Game*,Byte*);
short 	KvKR(Game*,Byte*);
short 	KQvK(Game*,Byte*);
short 	KvKQ(Game*,Byte*);
short 	KBNvK(Game*,Byte*);
short 	KvKBN(Game*,Byte*);
short 	KNPvK(Game*,Byte*);
short 	KvKNP(Game*,Byte*);
short 	KBPvK(Game*,Byte*);
short 	KvKBP(Game*,Byte*);
short 	KPvKB(Game*,Byte*);
short 	KBvKP(Game*,Byte*);
short 	KPvKN(Game*,Byte*);
short 	KNvKP(Game*,Byte*);	
short 	KQvKP(Game*,Byte*);
short 	KPvKQ(Game*,Byte*);
short 	KPvKP(Game*,Byte*);
short 	KRBvKR(Game*,Byte*);
short 	KRvKRB(Game*,Byte*);
short 	KRNvKR(Game*,Byte*);
short 	KRvKRN(Game*,Byte*);
short 	KBvKB(Game*,Byte*);
short 	KNvKN(Game*,Byte*);
short 	KRvKR(Game*,Byte*);
short 	KQvKQ(Game*,Byte*);
short 	KBPvKB(Game*,Byte*);
short 	KBvKBP(Game*,Byte*);
short 	KNPvKB(Game*,Byte*);
short 	KBvKNP(Game*,Byte*);
short 	KRvKB(Game*,Byte*);
short 	KBvKR(Game*,Byte*);
short 	KRvKN(Game*,Byte*);
short 	KNvKR(Game*,Byte*);
short 	KBvKN(Game*,Byte*);
short 	KNvKB(Game*,Byte*);	
short 	KRvKP(Game*,Byte*);
short 	KPvKR(Game*,Byte*);
short 	KRPvKR(Game*,Byte*);
short 	KRvKRP(Game*,Byte*);

int 	main()
{
 Game		Gm;
 Mvs		Mv;
 Byte		i,j,d;
 Dbyte		Mov;
 char 		Com[50],Pos[100],*Is;
 int		Iv;
 clock_t 	t1;
 BitMap		BM;
 
 char		Testpos[]="fen 8/5p1p/1p2pPk1/2p1P3/PpP1K2b/4B3/7P/8 w - - 0 2 "; 
 
 struct 	Inp {char Str[5000]; Byte volatile inp;} Input={" ",0};		// structure for user/GUI input  
 pthread_t	Tid0; 														// thread ID for user input
 
 pthread_create(&Tid0, NULL, ScanInput, (void*)(&Input));				// create the thread for ascii input	   
 InitDataStructures();													// initialize global data
 //GetPosition(&Gm,Startpos); strcpy(Pos,Startpos);						// default is startpos
 
 GetPosition(&Gm,Testpos); strcpy(Pos,Testpos);
 
 SetGlobalDefaults();													// set global variables to default values														

 while(1)
 {
  usleep(10000);														// sleep 1/100s to prevent busy thread
  switch(Input.inp)														// main program loop
  {
   case 0:  break;
   case 1:	printf("readyok\n"); fflush(stdout); 						// default answer to ping "isready"
   											Input.inp=0; 	break;										
   case 2:  free(hash_t); free(phash_t); free(mhash_t); free(ehash_t);	// free hash table space
   			pthread_exit(NULL); 			Input.inp=0;	break;		// "quit" command from GUI
   case 3:  strncpy(Pos,Input.Str+13,100);
   			GetPosition(&Gm,Input.Str);		Input.inp=0;	break;		// get position and moves
   case 4:	printf("id name Astimate3\n");								// answer to "uci" request from GUI
		    printf("id author Dr. Axel Steinhage, Germany, 2021\n"); 	// engine and author name 
		    printf("option name ClearTT type button\n");				// Clear hash table button
		    for(i=1;i<sizeof(Options)/sizeof(Options[0]);i++)			// announce engine options
		     if(Options[i].Val)
			 	printf("option name %s type check default true\n",
				 									Options[i].Name);
			 else printf("option name %s type check default false\n",
			 										Options[i].Name);
		    for(i=0;i<sizeof(Paras)/sizeof(Paras[0]);i++)				// announce engine parameters
		     printf("option name %s type spin default %d min %d max %d\n",
			  Paras[i].Name,Paras[i].Val,Paras[i].Low,Paras[i].High);
			printf("uciok\n"); 											// this is an UCI engine
		    fflush(stdout); 				Input.inp=0;	break;
   case 5:	maxdepth=level=nmate=0; MAXNODES=0;	Input.inp=0;			// "go" command from GUI
			Wtime=Btime=Winc=Binc=Movetime=0; Movestogo=0;				// initialize values							
    		if((Is=strstr(Input.Str,"ponder"))) 
								Ponder=true; else Ponder=false;			// Ponder?
		    if((Is=strstr(Input.Str,"wtime"))) 
								sscanf(Is,"%s %d",Com,&Wtime);			// white time left
		    if((Is=strstr(Input.Str,"btime"))) 
								sscanf(Is,"%s %d",Com,&Btime);			// black time left
		    if((Is=strstr(Input.Str,"winc")))  
								sscanf(Is,"%s %d",Com,&Winc);			// white increment
		    if((Is=strstr(Input.Str,"binc")))  
								sscanf(Is,"%s %d",Com,&Binc);			// black increment
		    if((Is=strstr(Input.Str,"movestogo"))) 
								sscanf(Is,"%s %d",Com,&Movestogo);		// moves to go
			else level=2;												// sudden death
		    if((Is=strstr(Input.Str,"depth"))) 							// fixed depth
			 		{level=3; sscanf(Is,"%s %c",Com,&maxdepth);}
		    if((Is=strstr(Input.Str,"nodes"))) 							// max nodes
			 		{level=4; sscanf(Is,"%s %llu",Com,&MAXNODES);}
		    if((Is=strstr(Input.Str,"mate")))  							// mate search
			 		{level=5; sscanf(Is,"%s %c",Com,&nmate);}
		    if((Is=strstr(Input.Str,"movetime"))) 						// exact time per move
			 		{level=6; sscanf(Is,"%s %d",Com,&Movetime);}
		    if((Is=strstr(Input.Str,"infinite"))) level=1;				// infinite
		    IterateSearch(&Gm);											// start search
		    if((level==1)||(Ponder)) while(!Stop);						// do not stop in pondering or analysis mode
		    UncodeMove(Gm.Move2Make,Com);								// get best move
		    printf("bestmove %s ",Com);
		    if(Gm.Pmove) 												// ponder move exists
			 {UncodeMove(Gm.Pmove,Com); printf("ponder %-7s",Com);}		// get ponder move
			printf("\n"); fflush(stdout);					break; 
   case 7:	InitNewGame(&Gm); 				Input.inp=0; 	break;		// prepare new game
   case 8:	if(strstr(Input.Str,"ClearTT")) 							// clear hash tables
   							{ClearTables();	Input.inp=0;	break;}
   			for(i=1;i<sizeof(Options)/sizeof(Options[0]);i++)			// scan options
   			 if(Is=strstr(Input.Str,Options[i].Name))					// option found
   			  if(strstr(Input.Str,"true")) 	Options[i].Val=true;
   			  else 							Options[i].Val=false;
			for(i=0;i<sizeof(Paras)/sizeof(Paras[0]);i++)				// scan parameters
   			 if(Is=strstr(Input.Str,Paras[i].Name))						// parameter found
   			  if(sscanf(Is,"%s %s %d",Com,Com,&Iv)==3)					// "setoption Name value Iv"
			  	if((Iv>=Paras[i].Low)&&(Iv<=Paras[i].High))				// value is in the allowed range
				{
				 Paras[i].Val=Iv; switch(i)								// set value and handle special cases
				 {
				  case 0: free(hash_t);									// change TT size, free memory 
				  		  HEN=(BitMap)(Iv*0x100000)/32;					// number of TT entries
				  		  DEN=(BitMap)(Iv*0x100000)/16; 				// number of perft/divide table entries
				  		  hash_t=(Byte*)(malloc(Iv*0x100000+32));		// allocate memory for transposition table
						  break;
				  case 1: free(phash_t);								// change pawn hashtable size, free memory 
				  		  PEN=(BitMap)(Iv*0x100000)/40;					// number of pawn hasth table entries
				  		  phash_t=(Byte*)(malloc(Iv*0x100000+40));		// allocate memory for pawn hash table
				  		  break;
				  case 2: free(mhash_t);								// change material hashtable size, free memory 
				  		  MEN=(BitMap)(Iv*0x100000)/8;					// number of material hash table entries
				  		  mhash_t=(Byte*)(malloc(Iv*0x100000+8));		// allocate memory for material hash table
				  		  break;
				  case 3: free(ehash_t);								// change evaluation hashtable size, free memory 
				  		  EEN=(BitMap)(Iv*0x100000)/8;					// number of evaluation hash table entries
				  		  ehash_t=(Byte*)(malloc(Iv*0x100000+8));		// allocate memory for evaluation hash table
				  		  break;
				  default: break;
				 }
				}
   											Input.inp=0; 	break;
   case 50: sscanf(Input.Str,"%s %d",Com,&d); Input.inp=0;				// get parameters
  			Perft(&Gm,d); 									break;		// perft 			
   case 51: sscanf(Input.Str,"%s %d",Com,&d); Input.inp=0;				// get parameters
  			Divide(&Gm,d);									break;		// divide
   case 52: PrintMoves(&Gm);				Input.inp=0;	break;		// print all legal moves
   case 53: PrintPosition(&Gm); 			Input.inp=0; 	break;		// print board
   case 56: if((Mov=CodeMove(&Gm,Input.Str)))							// make move
   			 {Move(&Gm,Mov); PrintPosition(&Gm);}						// show new position
   		    else printf("illegal move!\n");	Input.inp=0; 	break;		// wrong move																
   case 57: if((j=Gm.Move_n))											// take back move
   			{
			 ParseFen(&Gm,Pos);											// setup initial position
			 for(i=0;i<j-1;i++)	Move(&Gm,(Gm.Moves[i]).Mov);			// parse moves made
		    }
			PrintPosition(&Gm); 			Input.inp=0; 	break;		// show new position	
   case 58: t1=clock();	printf("testing speed:\n");						// speedtest
   			for(BM=0;BM<10000000;BM++)
   			{
   			 GenMoves(&Gm,&Mv);	
   			}
			printf("time: %lf\n",(double)(clock()-t1)/CLOCKS_PER_SEC); 
												Input.inp=0; break;  			
  }
 }
 return(0);																// exit program
}

void 	*ScanInput(void *Pe)											// scan user/GUI input
{
 struct In {char Str[5000]; volatile Byte inp;} *Input;					// structure for user input
 
 Input=(struct In*)Pe;													// cast input pointer					 
 	
 while(1) 
 {
  usleep(10000);
  if(!(Input->inp))														// endless loop over input
  {
   fgets(Input->Str,4999,stdin); 				Input->inp = 0;			// read standard input
   if(!strncmp(Input->Str,"isready",7)) 		Input->inp = 1;			// ping
   if(!strncmp(Input->Str,"quit",4)) 	  		Input->inp = 2;			// leave program
   if(!strncmp(Input->Str,"position",8)) 		Input->inp = 3;			// position and moves input
   if(!strncmp(Input->Str,"ucinewgame",10)) 	Input->inp = 7;			// new game
   else if(!strncmp(Input->Str,"uci",3)) 		Input->inp = 4;			// uci compatible?
   if(!strncmp(Input->Str,"go",2))  	   		Input->inp = 5;			// start calculating
   if(!strncmp(Input->Str,"stop",4)) 	  		Stop=true;				// stop calculating	
   if(!strncmp(Input->Str,"setoption",9))		Input->inp = 8;			// configure options
   if(!strncmp(Input->Str,"perft",5))   		Input->inp = 50;		// perft		
   if(!strncmp(Input->Str,"divide",6))  		Input->inp = 51;		// divided perft
   if(!strncmp(Input->Str,"list",4))  			Input->inp = 52;		// list moves
   if(!strncmp(Input->Str,"show",4))	 		Input->inp = 53;		// print board
   if(isalpha(Input->Str[0]) && isdigit(Input->Str[1]) &&
      isalpha(Input->Str[2]) && isdigit(Input->Str[3]))
     							 				Input->inp = 56;		// move
   if(!strncmp(Input->Str,"back",4))   			Input->inp = 57;		// take back move
   if(!strncmp(Input->Str,"speed",5))   		Input->inp = 58;		// speedtest
  }
 }
 return NULL;
}

void	SetGlobalDefaults()												// set global variables to initial default values
{
 maxdepth=nmate=0; MAXNODES=0; level=6;	Ponder=false;					
 Wtime=Btime=Winc=Binc=0; Movetime=10000; Movestogo=0;
}

void 	ClearTables()													// clear all hashtables
{
 BitMap AM;
 
 for(AM=0;AM<32*HEN;AM++) 		hash_t[AM]=0; HASHFILL=0;				// clear transposition table
 for(AM=0;AM<   HEN;AM++) 		hash_t[32*AM+1]|=4;						// set first entry bit
 for(AM=0;AM<40*PEN;AM++) 		phash_t[AM]=0;							// clear pawn/king evaluation hashtable	
 for(AM=0;AM< 8*MEN;AM++)		mhash_t[AM]=0;							// clear material hash table
 for(AM=0;AM< 8*EEN;AM++)  		ehash_t[AM]=0;							// clear evaluation hash table
 for(AM=0;AM<   PVN;AM++) 			 Pv[AM]=0; 							// clear pv table
}

void 	InitNewGame(Game* Gm)											// initialize a new game
{
 Byte 	i,j,k;
 
 ClearTables();															// clear all hash tables
 for(i=0;i<64;i++) for(j=0;j<17;j++) for(k=0;k<2;k++) 
   Gm->Counter[k][j][i]=0;												// initialize counter move table
}

void 	GetPosition(Game* Gm,char* Mo)									// gets position information from input
{
 char* 	St;
 Dbyte 	Mov;
 Mvs  	Mv;
 Byte	i;
 
 if((St=strstr(Mo,"fen"))) ParseFen(Gm,St+4);							// read new position
 else ParseFen(Gm,Startpos);											// startposition
 if((St=strstr(Mo,"moves"))) while((St=strchr(St,' '))) 				// read moves of game
 {
  while(*++St==' ');													// remove leading spaces
  if((*St<'a') || (*St>'h')) break;										// finished
  if((Mov=CodeMove(Gm,St))) 											// no nullmove
  {
   Move(Gm,Mov); 														// make move
   (Gm->Moves[Gm->Move_n-1]).check=										// set check flag
   					TestCheck(Gm,(Gm->Officer[Gm->color][0]).square);	
  }
 }
 Gm->Move_r=Gm->Move_n;													// set root move number
 GenMoves(Gm,&Mv);														// generate moves
 Mv.o=0; Mv.s=3; Mv.flg=0x70; i=0;										// init move picker
 while(Gm->Rootmvs[i]=PickMove(Gm,&Mv)) {Gm->TREESIZE[i]=1; i++;}		// fill root move list
 return;		    
}

bool 	TestAttk(Game* Gm, Byte k, Byte t, Byte c)						// test if piece type k attacks higher piece  on square t 
{
 BitMap PM;
 
 switch(k)																// switch piece type
 {
  case 3: if(!(PM=Gm->POSITION[c][2])) 					   return false;// no enemy queens
    	if(SLIDE[t][0][Gm->ROTATED[0]>>shift[0][t]&255]&PM) return true;// rook attacks queen
		if(SLIDE[t][1][Gm->ROTATED[1]>>shift[1][t]&255]&PM) return true;
		  break;
  case 4: if(!(PM=(Gm->POSITION[c][2]|Gm->POSITION[c][3]))) return false;// no enemy queens and rooks
  		if(SLIDE[t][2][Gm->ROTATED[2]>>shift[2][t]&255]&PM) return true; // bishop attacks rook or queen
 		if(SLIDE[t][3][Gm->ROTATED[3]>>shift[3][t]&255]&PM) return true;
		  break;
  case 5: PM=(Gm->POSITION[c][2]|Gm->POSITION[c][3]);					 // enemy queens and rooks
		  if(PM&STEP[2][t]) 								return true; // knight attacks rooks or queens
		  break;
  case 6: PM=(Gm->POSITION[c][0]&(~Gm->POSITION[c][6]));				 // all enemy officers
		  if(PM&STEP[4-c][t])								return true; // pawn attacks officer
 }
 return false;	
}

bool 	TestCheck(Game* Gm, Byte k)										// tests if check on square k
{
 Byte c=1-Gm->color;
 BitMap PM;

 if(k==99) k=(Gm->Officer[Gm->color][0]).square;						// square is king's position
 if(STEP[2][k]&Gm->POSITION[c][5]) 						return true;	// knight checks
 if(STEP[4-c][k]&Gm->POSITION[c][6]) 					return true;	// pawn checks
 if(PM=Gm->POSITION[c][2]|Gm->POSITION[c][3])							// queens and rooks
 {								
  if(SLIDE[k][0][Gm->ROTATED[0]>>shift[0][k]&255]&PM) 	return true;	// horizontal
  if(SLIDE[k][1][Gm->ROTATED[1]>>shift[1][k]&255]&PM) 	return true;	// vertical
 }  
 if(PM=Gm->POSITION[c][2]|Gm->POSITION[c][4])							// queens and bishops
 {
  if(SLIDE[k][2][Gm->ROTATED[2]>>shift[2][k]&255]&PM) 	return true;	// diagonal 1
  if(SLIDE[k][3][Gm->ROTATED[3]>>shift[3][k]&255]&PM) 	return true;	// diagonal 2								
 }
 return false;
}

bool 	TestMCheck(Game* Gm, Mvs* Mv, Dbyte Move)						// test if move announces check
{         
 Byte 	from=Move&63;													// some discovered checks are not 
 BitMap TM=(A8<<((Move>>8)&63));										// detected (king and non capturing pawns)
 
 switch((Gm->Piece[Gm->color][from]).type)								// switch piece type
 {
  case 2: if(TM&(Mv->OCHK[1])) 	  					return true;		// queen announces check on diagonal
  case 3: if(TM&(Mv->OCHK[0]))		  				return true;		// queen/rook announce check
  		  if(Mv->DCHK&(A8<<from)) 					return true; break;	// rook announces discovered check
  case 4: if(TM&(Mv->OCHK[1])) 	  					return true;		// bishop announces check
  		  if(Mv->DCHK&(A8<<from)) 					return true; break;	// bishop announces discovered check
  case 5: if(TM&(Mv->OCHK[2])) 	  					return true;		// knight announces check
  		  if(Mv->DCHK&(A8<<from)) 					return true; break; // knight announces discovered check
  case 6: if(TM&(Mv->OCHK[3])) 						return true;		// pawn announces check
		  if((Mv->DCHK&(A8<<from))&&((from&7)!=((Move>>8)&7)))			// discovered check pawn captures 
		  											return true; break;	// pawn announces discovered check
 }
 return false;	
}

void 	PrintMoves(Game* Gm)											// prints list of moves
{
 Mvs  		Mv;
 Dbyte		Mov;
 char		Mo[10];
 Byte		i;
 
 GenMoves(Gm,&Mv);														// init position and generate moves
 Mv.o=0; Mv.s=3; Mv.flg=0xF4; i=0;										// init move picker
 printf("all legal moves:\n");
 while(Mov=PickMove(Gm,&Mv))											// parse moves
 {
  UncodeMove(Mov,Mo); printf("s=%3d, %3d: %s",Mv.s,++i,Mo);				// print all moves
  printf(" \n");
 }
 printf("\n"); 
} 

void 	PrintPosition(Game* Gm)											// print board
{
 Byte		i,j,k,l;
 Mvs  		Mv;
 char		Mo[10];
 BitMap		BM;
 const char *St="-KQRBNPkqrbnp";
 
 printf("\n");
 if(Options[3].Val)														// Debug mode
 {
  printf("color: %d castles: %X fifty: %d move#: %d ep: %d draw: %d check: %d\n",// print position info
   Gm->color,(Gm->Moves[Gm->Move_n]).castles,
   (Gm->Moves[Gm->Move_n]).fifty,Gm->Move_n,(Gm->Moves[Gm->Move_n]).ep,
   DrawTest(Gm),(Gm->Moves[Gm->Move_n]).check);									
  printf("moves made: "); 												// print moves
  for(i=0; i<Gm->Move_n; i++) 
  {
   UncodeMove((Gm->Moves[i]).Mov,Mo); 									// translate move
   printf("%s",Mo); if((Gm->Moves[i]).check) printf("+"); printf(" ");	// print move and check symbol 
  }
  printf("\nposition hash: %llu\n",(Gm->Moves[Gm->Move_n]).HASH);		// transpostition hash
  printf("material hash: %llu\n",Gm->MHASH);							// material hash
  printf("pawn hash: %llu\n",Gm->PHASH);								// pawn hash
  printf("material signature white: %X black: %X \n",Gm->Matsig&0x1F,	// material signature
   		 (Gm->Matsig>>5)&0x1F);
  printf("piece square opening white: %d black: %d endgame white: %d black: %d\n",
   (Gm->Psv[0]).Open,(Gm->Psv[1]).Open,(Gm->Psv[0]).End,(Gm->Psv[1]).End);// piece square values
  GenMoves(Gm,&Mv);
  printf("material value: %d overall value: %d\n",						// print evaluation
  	MatEval(Gm),Evaluation(Gm,&Mv,0,0));							
 }
 for(i=0,BM=1; i<64; i++, BM<<=1)										// print board
 {
   l=0;
   for(j=1;j<7;j++) for(k=0;k<2;k++) if(BM&Gm->POSITION[k][j]) l=j+k*6;
    printf("%1c ",*(St+l));
   if(!((i+1)&7)) printf("\n");
 }
 printf("\n");
}

void 	ParseFen(Game *Gm, char *fen)									// scans fen string and sets all variables
{
 Byte 	c,i,j,l,o;
 Dbyte  Fi,Mo;	
 char 	Col[1],Cas[6],Enp[4],X[3];
 BitMap BS;

 Gm->phase=0; Gm->Move_n=0; Gm->Matsig=0; Gm->Lastbest=0;				// initialize game parameters										
 (Gm->Moves[0]).HASH=0; Gm->PHASH=0; Gm->MHASH=0; BS=1;					// initialize hash values
 
 for(j=0;j<2;j++)														// initialize game structure
 {
  for(i=0;i<7;i++)  Gm->POSITION[j][i]=0;								// initialize position bitmaps
  for(i=0;i<16;i++) 
   (Gm->Officer[j][i]).type=(Gm->Officer[j][i]).square=0;				// initialize officer type/index representation
  for(i=0;i<64;i++) (Gm->Piece[j][i]).type=(Gm->Piece[j][i]).index=0;	// initialize piece square/index representation
  (Gm->Count[j]).officers=(Gm->Count[j]).pawns=0;						// initialize officer and pawn counters
  (Gm->Psv[j]).Open=(Gm->Psv[j]).End=0;									// initialize piece square values
 }
 for(i=0;i<4;i++) 	Gm->ROTATED[i]=0;									// initialize rotated bitmaps
 
 while(*fen==' ') fen++;												// remove leading spaces
 
 i=0; while((sscanf(fen++,"%1s",X)==1)&&(i<64))							// get next character
 {
  l=(Byte)strcspn("/12345678KQRBNPkqrbnp",X);							// identify character
  if(l<9) {i+=l; BS<<=l;} else											// i,S=current square
  {																		// occupied square
   if(l<15) {c=0; l-=8;} else {c=1; l-=14;}								// c=color
   Gm->POSITION[c][l]|=BS; Gm->POSITION[c][0]|=BS; Gm->ROTATED[0]|=BS;	// register piece position
   (Gm->Piece[c][i]).type=l;											// register type on square												
   for(j=1;j<4;j++)
    {o=shift[j+3][i]; Gm->ROTATED[j]|=o<128?BS<<o:BS>>(256-o);}			// fill rotated bitmaps
   if(l<6) ((Gm->Count[c]).officers)++; else ((Gm->Count[c]).pawns)++;	// increment piece counters
   (Gm->Psv[c]).Open+=Square[c][l-1][0][i];								// piece square material value opening
   (Gm->Psv[c]).End +=Square[c][l-1][1][i];								// piece square material value endgame
   Gm->MHASH+=RANDOM_B[c][l-1][0];										// add piece to material hash 
   if(l>1) Gm->Matsig|=(1<<(5*c+l-2));									// set piece in material signature
   if(l==2) Gm->phase+=4; if(l==3) Gm->phase+=2;						// game phase (q=4,r=2,b,n=1)
   if((l==4)||(l==5)) (Gm->phase)++;
   if((l==6)||(l==1)) Gm->PHASH^=RANDOM_B[c][l-1][i];					// update pawn evaluation hash
   (Gm->Moves[0]).HASH^=RANDOM_B[c][l-1][i];							// update position hash
   i++; BS<<=1;															// next square
  }
 }
 for(c=0;o=0,c<2;c++) for(l=1;l<7;l++) for(i=0;i<64;i++)
  if(((Gm->Piece[c][i]).type==l)&&(l<6))
  {
	(Gm->Officer[c][o]).type=l; (Gm->Officer[c][o]).square=i; 			// register type,square
    (Gm->Piece[c][i]).index=o; o++;										// register index
  }
 sscanf(fen,"%c %s %s %hd %hd",Col,Cas,Enp,&Fi,&Mo);					// scan additional position information
 if(Col[0]=='b') Gm->color=1; 
 else {Gm->color=0; (Gm->Moves[0]).HASH^=RANDOM_P[12];}					// color hash
 i=(Byte)strlen(Cas); j=0;
 if(strcspn(Cas,"K")<i) {j|=1; (Gm->Moves[0]).HASH^=RANDOM_P[8];}		// update position hash 
 if(strcspn(Cas,"Q")<i) {j|=2; (Gm->Moves[0]).HASH^=RANDOM_P[9];}		// with castle rights
 if(strcspn(Cas,"k")<i) {j|=4; (Gm->Moves[0]).HASH^=RANDOM_P[10];}
 if(strcspn(Cas,"q")<i) {j|=8; (Gm->Moves[0]).HASH^=RANDOM_P[11];}
 (Gm->Moves[0]).fifty=Fi; (Gm->Moves[0]).castles=j; Gm->Move_m=Mo;		// set castle, fifty and made moves
 l=(Byte)strcspn("abcdefgh",Enp); j=(Byte)strcspn("87654321",Enp+1);	// look for ep
 if(l+j<16) 															// ep found
 {
  i=l+8*j; 																// ep square
  if(STEP[4-Gm->color][i]&(Gm->POSITION[Gm->color][6]))					// pawn can capture ep
   {(Gm->Moves[0]).ep=i; (Gm->Moves[0]).HASH^=RANDOM_P[i&7];}			// update position hash with ep
 }
 else (Gm->Moves[0]).ep=0;												// no ep
}

void 	InitDataStructures()											// initializes data structures
{
 int r;
 
 HEN = (BitMap)(Paras[0].Val*0x100000)/32; 								// number of transposition table entries
 PEN = (BitMap)(Paras[1].Val*0x100000)/40;								// number of pawn table entries
 MEN = (BitMap)(Paras[2].Val*0x100000)/8;								// number of material table entries
 EEN = (BitMap)(Paras[3].Val*0x100000)/8;								// number of evaluation table entries
 DEN = (BitMap)(Paras[0].Val*0x100000)/16; 								// number of perft/divide table entries)
 
 hash_t =(Byte*)(malloc(Paras[0].Val*0x100000+32));						// allocate memory for transposition table
 phash_t=(Byte*)(malloc(Paras[1].Val*0x100000+40));						// allocate memory for pawn hash table
 mhash_t=(Byte*)(malloc(Paras[2].Val*0x100000+8));						// allocate memory for material hash table
 ehash_t=(Byte*)(malloc(Paras[3].Val*0x100000+8));						// allocate memory for evaluation hash table
 
 for(r=0;r<1024;r++) recog[r]=NoRecog;									// initialize recognizer function pointers
 
 // material signature: white Q=001,R=002,B=004,N=008,P=010  black Q=020,R=040,B=080,N=100,P=200
 
 recog[0x000]=KvK; 														// KvK endgame recognizer function pointer
 recog[0x108]=KNvKN;
 recog[0x084]=KBvKB;
 recog[0x042]=KRvKR;
 recog[0x021]=KQvKQ;
 recog[0x094]=KBPvKB; recog[0x284]=KBvKBP;
 recog[0x098]=KNPvKB; recog[0x304]=KBvKNP;
 
 recog[0x010]=KPvK;   recog[0x200]=KvKP;
 recog[0x008]=KNvK;   recog[0x100]=KvKN; 
 recog[0x004]=KBvK;   recog[0x080]=KvKB;
 recog[0x002]=KRvK;   recog[0x040]=KvKR;
 recog[0x006]=KRvK;   recog[0x0C0]=KvKR;								// KRBvK	and KvKRB
 recog[0x00A]=KRvK;   recog[0x140]=KvKR;								// KRNvK  	and KvKRN
 recog[0x00E]=KRvK;   recog[0x1C0]=KvKR;								// KRBNvK 	and KvKRBN
 recog[0x001]=KQvK;   recog[0x020]=KvKQ;
 recog[0x003]=KQvK;	  recog[0x060]=KvKQ;								// KQRvK  	and KvKQR
 recog[0x005]=KQvK;   recog[0x0A0]=KvKQ;								// KQBvK  	and KvKQB
 recog[0x009]=KQvK;   recog[0x120]=KvKQ;								// KQNvK  	and KvKQN
 recog[0x007]=KQvK;   recog[0x0E0]=KvKQ;								// KQRBvK 	and KvKQRB
 recog[0x00B]=KQvK;   recog[0x160]=KvKQ;								// KQRNvK 	and KvKQRN
 recog[0x00F]=KQvK;	  recog[0x1E0]=KvKQ;								// KQRBNvK	and KvKQRBN
 recog[0x00D]=KQvK;	  recog[0x1A0]=KvKQ;								// KQBNvK	and KvKQBN
 
 recog[0x00C]=KBNvK;  recog[0x180]=KvKBN;
 recog[0x018]=KNPvK;  recog[0x300]=KvKNP;
 recog[0x014]=KBPvK;  recog[0x280]=KvKBP;
 recog[0x204]=KBvKP;  recog[0x090]=KPvKB;
 recog[0x208]=KNvKP;  recog[0x110]=KPvKN;
 // recog[0x201]=KQvKP;  recog[0x030]=KPvKQ; 
 recog[0x210]=KPvKP;
 recog[0x046]=KRBvKR; recog[0x0C2]=KRvKRB;
 recog[0x04A]=KRNvKR; recog[0x142]=KRvKRN;
 
 recog[0x082]=KRvKB;  recog[0x044]=KBvKR;
 recog[0x102]=KRvKN;  recog[0x048]=KNvKR;
 recog[0x104]=KBvKN;  recog[0x088]=KNvKB;
 // recog[0x202]=KRvKP;  recog[0x050]=KPvKR;
 recog[0x052]=KRPvKR; recog[0x242]=KRvKRP;
  
}

short 	NoRecog(Game* Gm, Byte *flags)									// No recognizer available
{
 *flags=128;															// indicates no recognizer found 
 return 0;
}

void 	GenMoves(Game *Gm, Mvs *Mv)										// generate moves as bitmaps
{
 Byte i,c,s,t,cm,cas,ep;
 BitMap B0,B1,B2,B3,BA,BC,BE1,BE2,BK,BM,BN,BP,ATK[8],K;

 //stat[11][0]++;														// count calls of move genrator
 
 for(i=0;i<6;i++) 	Mv->OATK[i]=0;										// initialize opponent attacks
 for(i=0;i<8;i++) 	{ATK[i]=0; Mv->Prop[i]=0;}							// hor., vert. and diag. attacks, promotion info
 
 c=Gm->color; BK=Gm->POSITION[c][1]; BA=BC=0; cm=0;						// stm color and king position bitmap
 cas=(Gm->Moves[Gm->Move_n]).castles; ep=(Gm->Moves[Gm->Move_n]).ep;	// castles and ep
 
 for(i=0;i<(Gm->Count[1-c]).officers;i++)								// walk thru opposing officers
 {
  s=(Gm->Officer[1-c][i]).square; t=(Gm->Officer[1-c][i]).type;			// piece position and type
  switch(t)																// parse type
  {
   case 1: BM=STEP[1][s]; break;										// king	
   case 2: BM=SLIDE[s][0][Gm->ROTATED[0]>>shift[0][s]&255]; 
   		   ATK[0]|=BM; 													// queen horizontal attacks
   	       BN=SLIDE[s][1][Gm->ROTATED[1]>>shift[1][s]&255];
		   ATK[1]|=BN; BM|=BN; 											// queen vertical attacks													
   	       BN=SLIDE[s][2][Gm->ROTATED[2]>>shift[2][s]&255];
		   ATK[2]|=BN; BM|=BN; 											// queen diagonal 1 attacks
		   BN=SLIDE[s][3][Gm->ROTATED[3]>>shift[3][s]&255];
		   ATK[3]|=BN; BM|=BN; 											// queen diagonal 2 attacks
		   break;
   case 3: BM=SLIDE[s][0][Gm->ROTATED[0]>>shift[0][s]&255];
   		   ATK[0]|=BM; 													// rook horizontal attacks
   	       BN=SLIDE[s][1][Gm->ROTATED[1]>>shift[1][s]&255];
		   ATK[1]|=BN; BM|=BN; 											// rook vertical attacks
		   break;
   case 4: BM=SLIDE[s][2][Gm->ROTATED[2]>>shift[2][s]&255];
		   ATK[2]|=BM; 													// bishop diagonal 1 attacks
		   BN=SLIDE[s][3][Gm->ROTATED[3]>>shift[3][s]&255];
		   ATK[3]|=BN; BM|=BN; 											// bishop diagonal 2 attacks
		   break;
   case 5: BM=STEP[2][s]; break;										// knight 		   
  }
  Mv->OATK[t]|=BM; BA|=BM;												// all attacks of types, all attacks
  if(BM&BK) {cm+=64; BC|=A8<<s;}										// increase check count, store checker
 }
 BP=Gm->POSITION[1-c][6]; BM=PA&BP; BN=PH&BP;							// opponent's pawns positions
 if(c) {BM>>=9; BN>>=7;} else {BM<<=7; BN<<=9;}							// opponent's pawns attacks
 Mv->OATK[0]=(BA|=BM|BN); s=(Gm->Officer[c][0]).square;					// register all attacks, stm king pos
 if(BM&BK) {cm+=64; BC=A8<<(s-7+c*16);}									// increase check count, store checker
 if(BN&BK) {cm+=64; BC=A8<<(s-9+c*16);}									// increase check count, store checker
 BM=SLIDE[s][0][Gm->ROTATED[0]>>shift[0][s]&255];						// horizontal attack line from stm king position
 if(ATK[0]&BK) {BC|=(BM&ATK[0]&(~Gm->ROTATED[0])); BA|=BM&(~BC);}		// check: block trace and xray attacks			
 BE1=ATK[0]; BE2=BM; ATK[0]&=BM;										// save horizontal attacks for ep BE=(ATK[0]|BM);
 for(i=1;i<4;i++) 														// check attacks on stm king
 {
  BM=SLIDE[s][i][Gm->ROTATED[i]>>shift[i][s]&255];						// attack lines from king position
  if(ATK[i]&BK) {BC|=(BM&ATK[i]&(~Gm->ROTATED[0])); BA|=BM&(~BC);}		// check: block trace and xray attacks
  ATK[i]&=BM;															// pinned pieces (including opponent!)
 }
 Mv->ADES=STEP[1][s];													// king attacks
 if(Mv->OFFM[0]=(Mv->ADES&(~Gm->POSITION[c][0])&(~BA))) cm++;			// legal king moves, BA=all opponent attacks
 Mv->AMVS=Mv->OFFM[0];													// all moves of pieces that can move										
 if(cm<64) BC=-1;														// all target squares allowed		
 B0=ATK[1]|ATK[2]|ATK[3]; B1=ATK[0]|ATK[2]|ATK[3];						// optimized attack maps
 B2=ATK[0]|ATK[1]|ATK[3]; B3=ATK[0]|ATK[1]|ATK[2];
 Mv->PINS=ATK[0]|B0;													// all pinned pieces stm																		
 if(cm>127) 															// double check: only king can move
 {
  for(i=1;i<16;i++) Mv->OFFM[i]=0;										// clear all officer moves except king
  for(i=0;i<4;i++)  Mv->PAWM[i]=0;										// clear all pawn moves
  goto GEN_REST;
 }
 for(i=1;i<(Gm->Count[c]).officers;i++)									// walk thru stm officers
 {
  s=(Gm->Officer[c][i]).square; t=(Gm->Officer[c][i]).type;				// piece position and type
  BK=A8<<s; BM=0;														// initialize attacks
  switch(t)	
  {
   case 2: if(!(B0&BK))													// not pinned
   		   {
			BN=SLIDE[s][0][Gm->ROTATED[0]>>shift[0][s]&255];			// queen horizontal
			ATK[4]|=BN; BM|=BN;
		   }
		   if(!(B1&BK))													// not pinned
   		   {
			BN=SLIDE[s][1][Gm->ROTATED[1]>>shift[1][s]&255];			// queen vertical
			ATK[5]|=BN; BM|=BN;
		   }
		   if(!(B2&BK))													// not pinned
   		   {
			BN=SLIDE[s][2][Gm->ROTATED[2]>>shift[2][s]&255];			// queen diagonal 1
			ATK[6]|=BN; BM|=BN;
		   }
		   if(!(B3&BK))													// not pinned
   		   {
			BN=SLIDE[s][3][Gm->ROTATED[3]>>shift[3][s]&255];			// queen diagonal 2
			ATK[7]|=BN; BM|=BN;
		   }
		   break;									
   case 3: if(!(B0&BK))													// not pinned
 		   {
			BN=SLIDE[s][0][Gm->ROTATED[0]>>shift[0][s]&255];			// rook horizontal
			ATK[4]|=BN; BM|=BN;
		   }
		   if(!(B1&BK))													// not pinned
   		   {
			BN=SLIDE[s][1][Gm->ROTATED[1]>>shift[1][s]&255];			// rook vertical
			ATK[5]|=BN; BM|=BN;
		   }
		   break;
   case 4: if(!(B2&BK))													// not pinned
   		   {
			BN=SLIDE[s][2][Gm->ROTATED[2]>>shift[2][s]&255];			// bishop diagonal 1
			ATK[6]|=BN; BM|=BN;
		   }
		   if(!(B3&BK))													// not pinned
   		   {
			BN=SLIDE[s][3][Gm->ROTATED[3]>>shift[3][s]&255];			// bishop diagonal 2
			ATK[7]|=BN; BM|=BN;
		   }
		   break;
   case 5: if(!((B3|ATK[3])&BK))										// not pinned
   			BM=STEP[2][s];												// knight 
   		   break;
  }
  Mv->ADES|=BM;															// all attacks
  Mv->OFFM[i]=BM&BC&(~Gm->POSITION[c][0]);	
  if(Mv->OFFM[i]) {Mv->AMVS|=Mv->OFFM[i]; cm++;} 						// legal moves
 }
 BP=Gm->POSITION[c][6]&(~ATK[0]);										// pawns not pinned horizontally
 BK=BP&(~ATK[2])&(~ATK[3]);												// pawns not pinned diagonally
 if(c) {BM=(BK<<8)&(~Gm->ROTATED[0]); BN=(BM&P6)<<8;}					// pawns step and double step
 else  {BM=(BK>>8)&(~Gm->ROTATED[0]); BN=(BM&P3)>>8;}					// white
 if(Mv->PAWM[1]=BM&BC) cm|=16; 											// pawns move forward one step
 if(Mv->PAWM[3]=BN&BC&(~Gm->ROTATED[0])) cm|=16;						// pawns double step							
 BK=BP&(~ATK[1]); BM=PA&BK; BN=PH&BK;									// pawns not pinned vertically, no wraps	
 if(c) {BM=(BM&(~ATK[3]))<<7; BN=(BN&(~ATK[2]))<<9;} 					// pawn captures, not pinned diagonally
 else  {BM=(BM&(~ATK[2]))>>9; BN=(BN&(~ATK[3]))>>7;}					// white			
 Mv->ADES|=BM|BN; BK=Gm->POSITION[1-c][0]&BC;							// all attacks, occupied by opponent
 if(Mv->PAWM[0]=BM&BK) cm|=32; if(Mv->PAWM[2]=BN&BK) cm|=32;			// pawns capture towards a and h line
 if(ep)
 {
  BM&=(A8<<ep); BN&=(A8<<ep); 											// isolate ep captures								
  if(BM|BN)																// ep capture possible
  {
   BK=(A8<<(ep+8-16*c));												// ep pawn
   if(BK&(ATK[2]|ATK[3])) BM=BN=0;										// ep pawn "pinned" diagonally   
   if((BE1&BK)&&(BE2&(BK<<1))) BM=0;									// double pinned horizontally
   if((BE1&BK)&&(BE2&(BK>>1))) BN=0;									// double pinned horizontally
   if((BE2&BK)&&(BE1&(BK<<1))) BM=0;									// double pinned horizontally
   if((BE2&BK)&&(BE1&(BK>>1))) BN=0;									// double pinned horizontally
   if(BC&BK) {Mv->PAWM[0]|=BM; Mv->PAWM[2]|=BN; if(BM|BN) cm|=32;}		// add ep capture(s)
  }
 }
 Mv->AMVS|=(Mv->PAWM[0]|Mv->PAWM[1]|Mv->PAWM[2]);						// single move indicator
 
 GEN_REST:
 if(cas&0x0F) if(c)														// castles allowed, black
 {
  if((cas&4)&&(!(BA&CSBC))&&(!(Gm->ROTATED[0]&CSB))) Mv->OFFM[0]|=A8<<6;// kingside allowed, no check, free 
  if((cas&8)&&(!(BA&CLBC))&&(!(Gm->ROTATED[0]&CLB))) Mv->OFFM[0]|=A8<<2;// queenside allowed, no check, free 
 }
 else																	// white
 {
  if((cas&1)&&(!(BA&CSWC))&&(!(Gm->ROTATED[0]&CSW))) Mv->OFFM[0]|=A8<<62;// kingside allowed, no check, free 
  if((cas&2)&&(!(BA&CLWC))&&(!(Gm->ROTATED[0]&CLW))) Mv->OFFM[0]|=A8<<58;// queenside allowed, no check, free
 }  
 s=(Gm->Officer[1-c][0]).square;										// opposite king position
 ATK[0]=SLIDE[s][0][Gm->ROTATED[0]>>shift[0][s]&255];					// horizontal checks
 ATK[1]=SLIDE[s][1][Gm->ROTATED[1]>>shift[1][s]&255];					// vertical checks
 Mv->OCHK[0]=ATK[0]|ATK[1];												// rook/queen checks
 ATK[2]=SLIDE[s][2][Gm->ROTATED[2]>>shift[2][s]&255];					// diagonal 1 checks
 ATK[3]=SLIDE[s][3][Gm->ROTATED[3]>>shift[3][s]&255];					// diagonal 2 checks
 Mv->OCHK[1]=ATK[2]|ATK[3];												// bishop/queen checks
 Mv->OCHK[2]=STEP[2][s];												// knight checks
 Mv->OCHK[3]=STEP[4-c][s];												// pawn checks
 Mv->DCHK=((ATK[0]&ATK[4])|(ATK[1]&ATK[5])|(ATK[2]&ATK[6])|				// discovered check pieces
 		  (ATK[3]&ATK[7])); 
 Mv->PINO=(ATK[4]&ATK[0])|(ATK[5]&ATK[1])|(ATK[2]&ATK[6])|				// pinned pieces opponent
 		 (ATK[3]&ATK[7]);
 Mv->cp=cm;																// counter for pieces that can move
 return;
}

Dbyte 	PickMove(Game *Gm, Mvs *Mv)										// picks next move from movelist
{
 // flg: 0:positive/zero SEE, 1: SEE, 2:save square, 
 //		 4: bad captures, 5: quiet moves, 6: check, 7: counter moves
 BitMap BM,KM;
 Dbyte  Fr,To,Pr,Hm;
 Byte 	i,j,p,q;
 
 NMoveO:
 	
 if(Mv->o) while(BM=(Mv->CMB)&Mv->OFFM[Mv->o-1])						// parse officer moves filtered with CMB
 {
  BM&=-BM;																// isolate destination bit 
  Fr=(Gm->Officer[Gm->color][Mv->o-1]).square;							// origin
  To=(find_b[(BM^BM-1)%67]<<8)+Fr;										// destination, build move												
  if((Mv->flg&2)&&(Mv->OATK[0]&BM)) 									// move has no fitting SEE
   {Mv->CMB^=BM; if(!SEE(Gm,To,(short)(Mv->flg&1))) continue;}			// clear move from current bitmap
  Mv->OFFM[Mv->o-1]^=BM; return(To);									// clear move from database and deliver										
 }
 
 NMoveP:
 																		// loop through moves
 switch(Mv->s)															// move selector stage
 {
  case 0:   (Mv->s)++; if(Hm=Mv->Bestmove)					goto SMove;	// hash move
  case 1:   (Mv->s)++;
  			if((Gm->Move_n==Gm->Move_r)&&(Hm=Gm->Lastbest)) goto SMove;	// best move of previous iteration, not at ply >0		   
  case 2:   if((Gm->Move_n>Gm->Move_r)||(Gm->Threadn)) break; 			// deliver root moves, not for helpers or at ply >0
  			//if(Gm->Move_n>Gm->Move_r) break;
   			i=0; BM=0; while(Gm->Rootmvs[i])							// parse root moves	 
			{
  		     if((!(Gm->Rootmvs[i]&64))&&(Gm->TREESIZE[i]>BM))		    // find new move with largest tree 
			  {BM=Gm->TREESIZE[i]; j=i;} i++;							// backup candidate, next move 
		    }
 		    if(BM) return Gm->Rootmvs[j]; else return 0;  				// deliver move	or no more root moves		   
  case 3:   if(BM=Mv->PAWM[i=2]&PROM8) 			goto PMove;  Mv->s+=1;	// promotion captures left
  case 4:   if(BM=Mv->PAWM[i=0]&PROM8) 			goto PMove;  Mv->s+=1;	// promotion captures right
  case 5:   Mv->s+=1; if(BM=Mv->PAWM[i=1]&PROM8) goto PMove;  			// queen promotions
  case 6:   Mv->s+=1; 
  			if(!(Mv->CMB=Mv->AMVS&Gm->POSITION[1-Gm->color][0])) 		// no captures possible
  										   		{Mv->s+=9; break;}								   
		   	if(!(Mv->CMB&=~Gm->POSITION[1-Gm->color][6])) 				// no officer captures possible
		   										{Mv->s+=5; break;} 
  case 7:  	if(BM=Mv->PAWM[i=2]&(Mv->CMB)) 		goto PMove; break;		// pawn captures officer right	   									  
  case 8:  	if(BM=Mv->PAWM[i=0]&(Mv->CMB)) 		goto PMove; break;		// pawn captures officer left 
  case 9:  	Mv->s+=1; if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][2])) break;// no queen captures possible
		   	Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->flg&=~2;		// prepare capture queens		   
  case 10: 	while(--(Mv->o)) 											// parse officers < queen
  			 if(((Gm->Officer[Gm->color][Mv->o-1]).type>2)&& 
  			     (Mv->CMB=Gm->POSITION[1-Gm->color][2])) 
				   								goto NMoveO; Mv->s+=1;	// capture queens only								 	 
  case 11: 	Mv->s+=1; if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][3])) break;// no rook captures possible
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->flg&=~2;		// prepare capture rooks
  case 12: 	while(--(Mv->o)) 											// parse officers < rook
  			 if(((Gm->Officer[Gm->color][Mv->o-1]).type>3)&& 
  			     (Mv->CMB=Gm->POSITION[1-Gm->color][3])) 
				   								goto NMoveO; Mv->s+=1;	// capture rooks only
  case 13: 	if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][0]&(~Mv->OATK[0])))	
  												{Mv->s+=3;		break;}	// capture hanging possible?
		    Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare capture hanging pieces
			Mv->flg&=~2; Mv->s+=1; 
  case 14: 	while(--(Mv->o)) 											// parse officers
  			 if(Mv->CMB=Gm->POSITION[1-Gm->color][0]&(~Mv->OATK[0]))	// capture hanging		 
  					 			 				goto NMoveO; Mv->s+=1;	
  case 15: 	if(BM=Mv->PAWM[i=2]&(~Mv->OATK[0])) goto PMove;  Mv->s+=1;	// pawn captures hanging pawn right	  									  
  case 16: 	if(BM=Mv->PAWM[i=0]&(~Mv->OATK[0])) goto PMove;  Mv->s+=1;	// pawn captures hanging pawn left								  					
  case 17: 	if(BM=Mv->PAWM[i=1]&PROM8) 			goto PMove;  Mv->s+=1;	// rest of promotions
  case 18: 	Mv->o=(Gm->Count[Gm->color]).officers+1; 
			Mv->flg|=3; Mv->s+=1;										// prepare captures with positive SEE		   
  case 19: 	while(--(Mv->o)>1) 											// captures with positive SEE
  		     {Mv->CMB=Gm->POSITION[1-Gm->color][0]; goto NMoveO;} 
			Mv->s+=1;
  case 20: 	if(BM=Mv->PAWM[i=2]) 				goto PMove; Mv->s+=1;	// pawn captures right	   									  
  case 21: 	if(BM=Mv->PAWM[i=0]) 				goto PMove; Mv->s+=1;	// pawn captures left
  case 22:  Mv->s+=1; Mv->o=(Gm->Count[Gm->color]).officers+1;
  			Mv->flg&=~1; Mv->flg|=2;									// prepare captures	with zero SEE
  case 23:  while(--(Mv->o)>1)											// captures with zero SEE
  			 {Mv->CMB=Gm->POSITION[1-Gm->color][0]; goto NMoveO;} 
			Mv->s+=1;			
  case 24: 	Mv->s+=1; if(Hm=Gm->Killer[Gm->Move_n-Gm->Move_r][0][0])	// mate killer
  		     if(Gm->Killer[Gm->Move_n-Gm->Move_r][0][1]==				// right piece
			   (Gm->Piece[Gm->color][Hm&63]).type) goto SMove;
  case 25:  Mv->s+=1; if(Hm=Gm->Killer[Gm->Move_n-Gm->Move_r][1][0])	// killer 1
  		     if(Gm->Killer[Gm->Move_n-Gm->Move_r][1][1]==				// right piece
			   (Gm->Piece[Gm->color][Hm&63]).type) goto SMove;
  case 26:  Mv->s+=1; if(Hm=Gm->Killer[Gm->Move_n-Gm->Move_r][2][0])	// killer 2
  		     if(Gm->Killer[Gm->Move_n-Gm->Move_r][2][1]==				// right piece
			   (Gm->Piece[Gm->color][Hm&63]).type) goto SMove;
  case 27:  Mv->s+=1; if(Gm->Move_n&&(Mv->flg&0x80))					// counter move
  		    {
			 Fr=(Gm->Moves[Gm->Move_n-1]).to&63;						// previous destination
		 	 if((Gm->Piece[1-Gm->color][Fr]).type<6)					// previous mover was officer
			  Hm=Gm->Counter[Gm->color][(Gm->Piece[1-Gm->color][Fr]).index][Fr];// get counter move
			 else Hm=Gm->Counter[Gm->color][16][Fr];
			 if(Hm) 								goto SMove;			// try counter move  
		    }									
  case 28:  while(KM=Mv->DCHK&Gm->POSITION[Gm->color][6])	 			// pawns that announce discovered check
		    {
			 KM&=-KM;													// one pawn at a time
			 if(((Gm->Officer[1-Gm->color][0]).square&7)!=				// opposing king not on same file as pawn
			 							(find_b[(KM^KM-1)%67]&7))		
			 {										
			  if(BM=((KM>>(8-16*Gm->color))&Mv->PAWM[i=1])) goto PMove;	// pawn moves forward												
			  if(BM=((KM>>(16-32*Gm->color))&Mv->PAWM[i=3]&PDS))		// double step
			  												goto DMove;	// deliver pawn move
			 }
			 Mv->DCHK-=KM;												// clear pawn from discovered check pieces
		    }
		    Mv->s+=1; Mv->o=(Gm->Count[Gm->color]).officers+1;			// officers announce discovered check? 
			Mv->flg&=~2; 			
  case 29:  while((--(Mv->o))&&(Mv->DCHK&Gm->POSITION[Gm->color][0])) 	// next discovered check officer
  		     if(Mv->DCHK&(A8<<(Gm->Officer[Gm->color][Mv->o-1]).square))// officer announces discovered check
  		     					{Mv->CMB=-1; goto NMoveO;} Mv->s+=1;
  case 30:  Mv->CMB=Mv->PAWM[i=3]&PDS&Mv->OCHK[3];		Mv->s+=1;		// pawn double step checks
  case 31:  if(Mv->CMB) goto DMoveS; 									// parse all pawn double steps checks
  			Mv->CMB=Mv->PAWM[i=1]&Mv->OCHK[3];			Mv->s+=1;		// pawn single step checks	
  case 32:  if(Mv->CMB) goto PMoveS;									// parse all pawn move forward checks
  		    Mv->o=(Gm->Count[Gm->color]).officers+1;	Mv->s+=1; 
  case 33:  while(--(Mv->o)>1)											// king cannot announce check directly
  		    {															// checks to save squares
			 j=(Gm->Officer[Gm->color][Mv->o-1]).type;					// piece type														
  		     if(j==5) Mv->CMB=Mv->OCHK[2];								// knight checks
		     else if(j==4) Mv->CMB=Mv->OCHK[1];							// bishop checks
		     else {Mv->CMB=Mv->OCHK[0]; if(j==2) Mv->CMB|=Mv->OCHK[1];} // rook or queen
  		     Mv->flg&=~1; Mv->flg|=2; 									// zero SEE
			 if(Mv->CMB&=Mv->OFFM[Mv->o-1]) 		goto NMoveO;		// check to SEE save squares
  		    }						 					Mv->s+=1;
  case 34:  if(BM=Mv->PAWM[i=1]) while(BM)								// parse pawn moves forward
  		    {
  		  	 KM=BM&(-BM); q=find_b[(KM^KM-1)%67];						// destination of pawn
  		  	 if(!(PAWN_E[Gm->color][1][q]&Gm->POSITION[1-Gm->color][6]))// passed pawn?
			 										goto PMove;
		     BM-=KM;
		    } 							 						break;		    
  case 35:  Mv->CMB=PST[Gm->color][Gm->phase<16?1:0][5][0]&				// prepare pawn double step to save squares...
  			 Mv->PAWM[3]&PDS;									break;	// ... with good pst entries	
  case 36:  if(Mv->CMB) 							goto DMoveS;		// parse all pawn double steps
  		    Mv->CMB=PST[Gm->color][Gm->phase<16?1:0][5][0]&				// prepare pawn single step to save squares						
			 Mv->PAWM[1];										break;	
  case 37:  if(Mv->CMB) 							goto PMoveS;		// parse all pawn move forward
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare officer moves to save squares
			Mv->flg&=~1; Mv->flg|=2; Mv->s+=1;
  case 38:  while(--(Mv->o)) 											// parse officers
  		    {
  		   	 i=(Gm->Officer[Gm->color][(Mv->o)-1]).type;				// piece type
  		   	 Mv->CMB=PST[Gm->color][Gm->phase<16?1:0][i][0]; goto NMoveO;// good pst squares				
	        }										break;			 
  case 39:  Mv->CMB=PST[Gm->color][Gm->phase<16?1:0][5][1]&				// prepare pawn double step to save squares...
  			 Mv->PAWM[3]&PDS;									break;	// ... with average pst entries	
  case 40:  if(Mv->CMB) 							goto DMoveS;		// parse all pawn double steps
  		    Mv->CMB=PST[Gm->color][Gm->phase<16?1:0][5][1]&				// prepare pawn single step to save squares						
			 Mv->PAWM[1];										break;	
  case 41:  if(Mv->CMB) 							goto PMoveS;		// parse all pawn move forward
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare officer moves to save squares
			Mv->flg&=~1; Mv->flg|=2; Mv->s+=1;
  case 42:  while(--(Mv->o)) 											// parse officers
  		    {
  		   	 i=(Gm->Officer[Gm->color][(Mv->o)-1]).type;				// piece type
  		   	 Mv->CMB=PST[Gm->color][Gm->phase<16?1:0][i][1]; goto NMoveO;// average pst squares				
	        }													break;
  case 43:  Mv->CMB=Mv->PAWM[3]&PDS;							break;	// prepare pawn double step to save squares
  case 44:  if(Mv->CMB) goto DMoveS;	Mv->CMB=Mv->PAWM[1];	break;  // double step to save squares
  case 45:  if(Mv->CMB) goto PMoveS;									// parse all pawn move forward to save squares
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; 
			Mv->flg&=~1; Mv->flg|=2; Mv->s+=1;							// prepare officer moves to save squares
  case 46:  while(--(Mv->o)) {Mv->CMB=-1; goto NMoveO;} 		break; 	// officer moves to save squares 
  case 47:  Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare rest of checks
  			Mv->flg&=~2; Mv->s+=1;										// no SEE	
  case 48:  while(--(Mv->o)>1)											// king cannot announce check directly
  		    {
			 j=(Gm->Officer[Gm->color][Mv->o-1]).type;					// piece type									
  		     if(j==5) Mv->CMB=Mv->OCHK[2];								// knight checks
		     else if(j==4) Mv->CMB=Mv->OCHK[1];							// bishop checks
		     else {Mv->CMB=Mv->OCHK[0]; if(j==2) Mv->CMB|=Mv->OCHK[1];} // rook or queen
  		     goto NMoveO;
  		    }										Mv->s+=1;
  case 49:  Mv->s+=1; Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->flg&=~2;// scan captures with negative SEE
  case 50:  while(--(Mv->o)>1) 											// king cannot move to attacked square
  			 {Mv->CMB=Gm->POSITION[1-Gm->color][0]; goto NMoveO;} 		// captures with negative SEE
		    Mv->s=201;											break;	// rest of moves			
						 		    
  case 100: if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][0]))				// quiescence moves 
  													{Mv->s+=17; break;}	// no captures possible
  		  	if(!Gm->Move_n)							{Mv->s+=3; break;}	// no previous move
  		  	Mv->CMB=(A8<<((Gm->Moves[Gm->Move_n-1]).to&63));			// destination of previous move
  		  	if(!(Mv->CMB&Mv->AMVS))					{Mv->s+=3; break;}	// capture on previous destination not possible
		  	Mv->s+=1; if(BM=Mv->PAWM[i=0]&Mv->CMB)	goto PMove; 		// pawn captures left
  case 101: Mv->s+=1; if(BM=Mv->PAWM[i=2]&Mv->CMB)	goto PMove;			// pawn captures right
  case 102: Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare officer captures
  			Mv->s+=1; Mv->flg&=~2; 		
  case 103: while(--(Mv->o)) 			  		goto NMoveO; Mv->s+=1;	// loop through officers			
  case 104: if(BM=Mv->PAWM[i=2]&PROM8) 			goto PMove;  Mv->s+=1;	// promotion captures left
  case 105: if(BM=Mv->PAWM[i=0]&PROM8) 			goto PMove;  Mv->s+=1;	// promotion captures right
  case 106: Mv->s+=1; if(BM=Mv->PAWM[i=1]&PROM8)  goto PMove; 			// queen promotions
  case 107: Mv->CMB=Gm->POSITION[1-Gm->color][0]&
  				  (~Gm->POSITION[1-Gm->color][6]);						// all oponent officers
  		    if(!(Mv->CMB&Mv->AMVS)) 	{Mv->s+=6; break;}   Mv->s+=1;	// no officer captures possible		   
  case 108: if(BM=Mv->PAWM[i=2]&(Mv->CMB)) 		goto PMove;  Mv->s+=1;	// pawn captures officer right	   									  
  case 109: if(BM=Mv->PAWM[i=0]&(Mv->CMB)) 		goto PMove;  Mv->s+=1;	// pawn captures officer left
  case 110: Mv->s+=1; if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][2])) break;// no queen captures possible
		    Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->flg&=~2; 		// prepare capture queens		    
  case 111: while(--(Mv->o)) 											// parse officers < queen
  			 if(((Gm->Officer[Gm->color][Mv->o-1]).type>2)&& 
  			     (Mv->CMB=Gm->POSITION[1-Gm->color][2])) 
				   								goto NMoveO; Mv->s+=1;	// capture queens only 								 	 
  case 112: Mv->s+=1; if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][3])) break;// no rook captures possible
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->flg&=~2;		// prepare capture rooks
  case 113: while(--(Mv->o)) 											// parse officers < rook
  			 if(((Gm->Officer[Gm->color][Mv->o-1]).type>3)&& 
  			     (Mv->CMB=Gm->POSITION[1-Gm->color][3])) 
				   								goto NMoveO; Mv->s+=1;	// capture rooks only		    
  case 114: if(!(Mv->AMVS&Gm->POSITION[1-Gm->color][0]&(~Mv->OATK[0])))	
  												{Mv->s+=3;		break;}	// capture hanging possible?
		    Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare capture hanging pieces
			Mv->flg&=~2; Mv->s+=1; 
  case 115: while(--(Mv->o)) 											// parse officers
  			 if(Mv->CMB=Gm->POSITION[1-Gm->color][0]&(~Mv->OATK[0]))	// capture hanging		 
  					 			 				goto NMoveO; Mv->s+=1;	
  case 116: if(BM=Mv->PAWM[i=2]&(~Mv->OATK[0])) goto PMove;  Mv->s+=1;	// pawn captures hanging pawn right	  									  
  case 117: if(BM=Mv->PAWM[i=0]&(~Mv->OATK[0])) goto PMove;  Mv->s+=1;	// pawn captures hanging pawn left
   
  case 118: if(BM=Mv->PAWM[i=1]&PROM8) 			goto PMove;  Mv->s+=1;	// rest of promotions
  			
  case 119: Mv->o=(Gm->Count[Gm->color]).officers+1; 
			Mv->flg|=3; Mv->s+=1;										// prepare captures with positive SEE		   
  case 120: while(--(Mv->o)>1) 											// captures with positive SEE
  		     {Mv->CMB=Gm->POSITION[1-Gm->color][0]; goto NMoveO;} 
			Mv->s+=1;
  case 121: if(BM=Mv->PAWM[i=2]) 				goto PMove; Mv->s+=1;	// pawn captures right	   									  
  case 122: if(BM=Mv->PAWM[i=0]) 				goto PMove; Mv->s+=1;	// pawn captures left
			
  case 123: Mv->s+=1; Mv->o=(Gm->Count[Gm->color]).officers+1;
  			Mv->flg&=~1; Mv->flg|=2;									// prepare captures	with zero SEE
  case 124: while(--(Mv->o)>1)											// captures with zero SEE
  			 {Mv->CMB=Gm->POSITION[1-Gm->color][0]; goto NMoveO;} 
			Mv->s+=1;

  case 125: if(!(Mv->flg&0x40)) 					{Mv->s+=6; break;} 	// no checks
  		    while(KM=Mv->DCHK&Gm->POSITION[Gm->color][6])	 			// pawns that announce discovered check
		    {
			 KM&=-KM;													// one pawn at a time
			 if(((Gm->Officer[1-Gm->color][0]).square&7)!=				// opposing king not on same file as pawn
			 							(find_b[(KM^KM-1)%67]&7))		
			 {										
			  if(BM=((KM>>(8-16*Gm->color))&Mv->PAWM[i=1])) goto PMove;	// pawn moves forward												
			  if(BM=((KM>>(16-32*Gm->color))&Mv->PAWM[i=3]&PDS))		// double step
			  												goto DMove;	// deliver pawn move
			 }
			 Mv->DCHK-=KM;												// clear pawn from discovered check pieces
		    }
		    Mv->s+=1; Mv->o=(Gm->Count[Gm->color]).officers+1;			// officers announce discovered check? 
			Mv->flg&=~2;			
  case 126: while((--(Mv->o))&&(Mv->DCHK&Gm->POSITION[Gm->color][0])) 	// next discovered check officer
  		     if(Mv->DCHK&(A8<<(Gm->Officer[Gm->color][Mv->o-1]).square))// officer announces discovered check
  		     					{Mv->CMB=-1; goto NMoveO;} Mv->s+=1;
 
  case 127: if(BM=Mv->PAWM[i=3]&PDS&Mv->OCHK[3]) goto DMove; Mv->s+=1;	// parse all pawn double steps checks
  case 128: if(BM=Mv->PAWM[i=1]&Mv->OCHK[3]) 	 goto PMove;			// parse all pawn move forward checks
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->s+=1;
  case 129: while(--(Mv->o)>1)											// king cannot announce check directly
  		    {															// checks to save squares
			 j=(Gm->Officer[Gm->color][Mv->o-1]).type;					// piece type														
  		     if(j==5) Mv->CMB=Mv->OCHK[2];								// knight checks
		     else if(j==4) Mv->CMB=Mv->OCHK[1];							// bishop checks
		     else {Mv->CMB=Mv->OCHK[0]; if(j==2) Mv->CMB|=Mv->OCHK[1];}  // rook or queen
  		     Mv->flg&=~1; Mv->flg|=2; 									// zero SEE
			 if(Mv->CMB&=Mv->OFFM[Mv->o-1]) 		goto NMoveO;		// check to SEE save squares
  		    }						 				Mv->s+=1;
  case 130: Mv->o=(Gm->Count[Gm->color]).officers+1; 					// prepare rest of checks
  			Mv->flg&=~2; Mv->s+=1;										// no SEE	
  case 131: while(--(Mv->o)>1)											// king cannot announce check directly
  		    {
			 j=(Gm->Officer[Gm->color][Mv->o-1]).type;					// piece type									
  		     if(j==5) Mv->CMB=Mv->OCHK[2];								// knight checks
		     else if(j==4) Mv->CMB=Mv->OCHK[1];							// bishop checks
		     else {Mv->CMB=Mv->OCHK[0]; if(j==2) Mv->CMB=Mv->OCHK[1];}  // rook or queen
  		     goto NMoveO;
  		    }										Mv->s+=1;
  case 132: Mv->s+=1; if(!(Mv->flg&0x10))	 		break;				// scan captures with negative SEE?
  		    Mv->o=(Gm->Count[Gm->color]).officers+1; Mv->flg&=~2;		// prepare rest of captures	
  case 133: while(--(Mv->o)>1) 											// king cannot move to attacked square
  			 {Mv->CMB=Gm->POSITION[1-Gm->color][0]; goto NMoveO;} 		// captures with negative SEE
		    Mv->s+=1;	
    
  case 134: if(!(Mv->flg&0x20)) return(0);			Mv->s=201; break;	// no quiet moves
  		   			
  case 200: if(BM=Mv->PAWM[i=2])					goto PMove; break;	// unsorted: pawn capture left
  case 201: if(BM=Mv->PAWM[i=0])					goto PMove; break;	// unsorted: pawn capture right
  case 202: if(BM=Mv->PAWM[i=3]) 					goto DMove;	break;	// unsorted: pawn double step
  case 203: if(BM=Mv->PAWM[i=1]) 	   				goto PMove; break;	// unsorted: pawn single step
  case 204: Mv->o=(Gm->Count[Gm->color]).officers+1;
   			Mv->s+=1; Mv->flg&=~2; 										// prepare officer moves
  case 205: if(--(Mv->o)) 			 {Mv->CMB=-1; goto NMoveO;} break;	// officer moves
  case 250: Mv->s++; if(Hm=Mv->Bestmove)				   goto SMove;	// test move
  default: return(0);  
 }
 (Mv->s)+=1; (Mv->o)=0; goto NMoveP;									// next move
 
 DMove:																	// pawn double step 
  BM&=-BM; Mv->PAWM[3]^=BM; Mv->o=0;									// clear move
  To=find_b[(BM^BM-1)%67]; Fr=To+16-32*Gm->color; 						// destination and origin
  return((To<<8)+Fr);													// deliver move
 
 DMoveS:																// pawn double step SEE save
  BM=Mv->CMB; To=find_b[(BM^BM-1)%67]; Fr=To+16-32*Gm->color; Mv->o=0;	// from and to square
  Mv->CMB&=Mv->CMB-1; Fr+=(To<<8);  									// clear move
  if(SEE(Gm,Fr,0)) {BM&=-BM; (Mv->PAWM[3])^=BM; return Fr;} goto NMoveP;// deliver move if save
   
 PMove:																	// pawn regular move
  BM&=-BM; To=find_b[(BM^BM-1)%67]; Fr=To+9-i-16*Gm->color; Mv->o=0;	// isolate move, from and to square			
  if(BM&PROM8)															// promotion
  {
   Pr=((Mv->Prop[Fr&7])>>(4*i))&15;										// bitmap of qrbn										
   for(j=1,q=0;q<4;j<<=1,q++)
    if(!(Pr&j)) {Pr|=j; To+=(q<<6); break;}								// choose free piece
   if(Pr==15) Mv->PAWM[i]^=BM;											// last piece selected, clear move
   else (Mv->Prop[Fr&7])|=(Pr<<(4*i));									// save promotion bitmap
   Fr+=128;																// indicate promotion
  }
  else Mv->PAWM[i]^=BM;													// clear move from database
  return((To<<8)+Fr);													// deliver move	
 
 PMoveS:																// pawn single step SEE save
  BM=Mv->CMB; To=find_b[(BM^BM-1)%67]; Fr=To+8-16*Gm->color; Mv->o=0;	// from and to square
  Mv->CMB&=(Mv->CMB)-1; Fr+=(To<<8);									// clear move
  if(SEE(Gm,Fr,0)) {BM&=-BM; (Mv->PAWM[1])^=BM; return Fr;} goto NMoveP;// deliver move if save
  
 SMove:																	// predefined move
  Mv->o=0; Mv->flg&=~2; Fr=Hm&63;										// initialize move search
  if(j=(Gm->Piece[Gm->color][Fr]).type)									// piece on origin exists
  { 
   Mv->CMB=(A8<<(To=((Hm>>8)&63)));										// bitmap of destination
   if(j<6) {Mv->o=(Gm->Piece[Gm->color][Fr]).index+1; goto NMoveO;}		// officer
   if((To-Fr==16)||(To-Fr==-16))										// double step pawn
  	if(BM=(Mv->PAWM[3]&(Mv->CMB))) 	goto DMove; else goto NMoveP;		// check if move is possible
   i=9-16*Gm->color+To-Fr;												// index of pawn move bitmap
   if((i<3)&&(BM=(Mv->PAWM[i]&(Mv->CMB))))								// ordinary pawn move
   {
	BM&=-BM; if(!(BM&PROM8)) {Mv->PAWM[i]^=BM; return(Hm&0xFFBF);}		// no promotion but move possible
	Pr=((Mv->Prop[Fr&7])>>(4*i))&15;									// bitmap of qrbn										
  	p=1; p<<=(Hm>>14);													// number and bitmap of promoted piece (0..3)
	if(!(Pr&p))															// piece is free
	{
	 Pr|=p;	if(Pr==15) Mv->PAWM[i]^=BM;									// set bit of selected piece, last piece selected
	 else (Mv->Prop[Fr&7])|=(Pr<<(4*i));								// store finished piece									 
   	 return(Hm&0xFFBF);													// promotion move possible
	}
   }
  }
  goto NMoveP;															// move not possible}
}

Dbyte 	CodeMove(Game* Gm, char *Mo)									// translates move from string
{
 Mvs	Mv;
 Byte	Fr,To;
 Dbyte	Mov;
 
 GenMoves(Gm,&Mv);
 To=(Byte)(Mo[2]-'a'+8*('8'-Mo[3])); Fr=(Byte)(Mo[0]-'a'+8*('8'-Mo[1]));// get destination and origin
 Mv.o=0; Mv.s=200; Mv.flg=0;											// init move picker
 while(Mov=PickMove(Gm,&Mv))											// get the moves
  if(((Byte)(Mov&63)==Fr)&&((Byte)((Mov>>8)&63)==To)) break;			// compare squares
 if(Mov)																// no nullmove 
 {
  if(Mo[4]=='r') Mov+=0x4000;											// rook promotion
  else if(Mo[4]=='b') Mov+=0x8000;										// bishop promotion
  else if(Mo[4]=='n') Mov+=0xC000;										// knight promotion
 }
 return(Mov);	
}

void 	UncodeMove(Dbyte Mv, char *Mo)									// translates move to string
{
 Byte from,to,pro;
 const char *M1="0000";													// nullmove
 const char *Po="kqrbnpabcdefgh12345678x+#=-";							// all move symbols

 *Mo='\0'; from=(Byte)(Mv&63); to=(Byte)((Mv>>8)&63);					// split move into origin and destination
 pro=(Byte)((Mv>>14)&3);												// promotion piece

 if(from==to) {strcat(Mo,M1); return;}									// nullmove
 strncat(Mo,Po+6+from%8,1); strncat(Mo,Po+21-from/8,1);					// transform origin 
 strncat(Mo,Po+6+to%8,1); strncat(Mo,Po+21-to/8,1);						// transform destination
 if(Mv&128) strncat(Mo,Po+pro+1,1);										// promotion
 if(!Options[0].Val) if(Mv&64) strcat(Mo,"+");							// non-uci: draw check symbol
}

void 	Move(Game* Gm, Dbyte Mv)										// makes move
{
 Byte 	i,j,c,cas,ep,fi,f,t,cs,p,o,s,pc,oc;
 BitMap	HB,TB,FB,BS;
 
 c=Gm->color; HB=(Gm->Moves[Gm->Move_n]).HASH;							// stm color and hash
 cas=(Gm->Moves[Gm->Move_n]).castles; ep=(Gm->Moves[Gm->Move_n]).ep;	// castles and ep
 fi=Gm->Moves[Gm->Move_n].fifty;										// fifty move counter
 cs=t=(Byte)(Mv>>8)&63; f=(Byte)(Mv&63);								// destination and origin
 (Gm->Moves[Gm->Move_n]).from=f; (Gm->Moves[Gm->Move_n]).to=t;			// register move in move list
 if(Mv&64)  (Gm->Moves[Gm->Move_n]).check=true;							// set check info
 else		(Gm->Moves[Gm->Move_n]).check=false;
 if(Mv&128) (Gm->Moves[Gm->Move_n]).prom=(Byte)(Mv>>14)+2;				// promoted piece
 else		(Gm->Moves[Gm->Move_n]).prom=0;
 (Gm->Moves[Gm->Move_n]).Mov=Mv;										// compressed move
 p=(Gm->Piece[c][f]).type; o=(Gm->Piece[c][f]).index;					// piece type and index 
 (Gm->Psv[c]).Open+=Square[c][p-1][0][t]-Square[c][p-1][0][f];			// piece square value opening
 (Gm->Psv[c]).End +=Square[c][p-1][1][t]-Square[c][p-1][1][f];			// piece square value endgame
 (Gm->Move_n)++; fi++; Gm->color=1-Gm->color; HB^=RANDOM_P[12];			// change color, update color hash			
 if(ep)																	// old position had ep square
 {													
  if((p==6)&&(t==ep)) 													// pawn captures ep
  {
   cs=t+8-16*c; TB=A8<<cs; Gm->ROTATED[0]-=TB;							// captured ep pawn
   for(i=1;i<4;i++)														// remove pawn from rotated bitmaps
    {s=shift[i+3][cs]; Gm->ROTATED[i]-=s<128?TB<<s:TB>>(256-s);}		// update rotated bitmaps after pawn is gone
  }
  HB^=RANDOM_P[ep&7]; ep=0;												// undo ep hash and clear in new position
 }
 if(f==t) {fi=0; goto Finish;}											// nullmove
 FB=A8<<f; TB=A8<<t;													// bitmaps of move
 (Gm->Piece[c][t]).type=p; (Gm->Piece[c][t]).index=o;					// set piece on destination square
 (Gm->Piece[c][f]).type=(Gm->Piece[c][f]).index=0;						// clear piece from origin square
 Gm->POSITION[c][0]^=TB|FB; Gm->POSITION[c][p]^=TB|FB; 					// move piece in type-bitmaps
 Gm->ROTATED[0]-=FB; Gm->ROTATED[0]|=TB;								// move piece in general bitmaps
 for(i=1;i<4;i++)														// move piece in rotated bitmaps
 {			
  s=shift[i+3][f]; Gm->ROTATED[i]-=s<128?FB<<s:FB>>(256-s);
  s=shift[i+3][t]; Gm->ROTATED[i]|=s<128?TB<<s:TB>>(256-s);	 
 }
 HB^=RANDOM_B[c][p-1][f]; HB^=RANDOM_B[c][p-1][t];						// update hash for moving piece
 if((p==6)||(p==1))														// pawn or king move
  {Gm->PHASH^=RANDOM_B[c][p-1][f]; Gm->PHASH^=RANDOM_B[c][p-1][t];}		// update pawn evaluation hash
 if(pc=(Gm->Piece[1-c][cs]).type)										// captured piece type
 {	
  oc=(Gm->Piece[1-c][cs]).index;										// captured piece index
  (Gm->Moves[Gm->Move_n-1]).cap=(oc<<4)+pc;								// index and type of captured piece
  if(cs!=t)	(Gm->Moves[Gm->Move_n-1]).cap|=8;							// ep		
  (Gm->Piece[1-c][cs]).type=(Gm->Piece[1-c][cs]).index=0;				// remove piece from board	
  Gm->POSITION[1-c][0]-=(A8<<cs); Gm->POSITION[1-c][pc]-=(A8<<cs); 		// remove piece from type-bitmaps							
  if(!Gm->POSITION[1-c][pc]) Gm->Matsig&=~(0x001<<(5-5*c+pc-2));		// no piece of this type left? remove from matsig	
  HB^=RANDOM_B[1-c][pc-1][cs];											// update position hash
  Gm->MHASH-=RANDOM_B[1-c][pc-1][0];									// update material hash
  (Gm->Psv[1-c]).Open-=Square[1-c][pc-1][0][cs];						// piece square value opening
  (Gm->Psv[1-c]).End -=Square[1-c][pc-1][1][cs];						// piece square value endgame													
  fi=0;																	// reset fifty move counter 													
  if(pc==6) 															// pawn is captured
   {((Gm->Count[1-c]).pawns)--;  Gm->PHASH^=RANDOM_B[1-c][5][cs];}		// decrement pawn counter and update pawn hash
  else
  {
	i=--((Gm->Count[1-c]).officers);									// decrement officer counter
	if(pc==2) Gm->phase-=4; else if(pc==3) Gm->phase-=2; 				// update game phase: queen or rook 
	else (Gm->phase)--;													// bishop or knight
	if(i!=oc)															// captured not last in list									
	{
	 (Gm->Officer[1-c][oc]).type=(Gm->Officer[1-c][i]).type;			// replace captured by last in list
	 j=(Gm->Officer[1-c][oc]).square=(Gm->Officer[1-c][i]).square;
	 (Gm->Piece[1-c][j]).index=oc;										// update index 							
	}
	(Gm->Officer[1-c][i]).type=(Gm->Officer[1-c][i]).square=0;			// clear last in list		
  }													
 }
 else (Gm->Moves[Gm->Move_n-1]).cap=0;									// no capture
 if(p==6)																// pawn move													
 {
  if(((t-f==16)||(f-t==16))&&											// pawn double step
  	  (STEP[3+c][t+8-16*c]&Gm->POSITION[1-c][6])) 						// oposing pawn can capture ep						
   {ep=t+8-16*c; HB^=RANDOM_P[ep&7];}									// set ep hash only if ep possible 				
  if(Mv&128)															// promotion
  {
   p=(Byte)(Mv>>14)+2; Gm->Matsig|=(1<<(3+p-5+5*c));					// officer type, set material signature  
   if(p==2) Gm->phase+=4; else if(p==3) Gm->phase+=2; else (Gm->phase)++;// update game phase
   ((Gm->Count[c]).pawns)--; i=((Gm->Count[c]).officers)++;				// dec. pawn, inc. officer
   Gm->POSITION[c][6]-=TB; Gm->POSITION[c][p]|=TB;						// remove pawn, set officer		
   if(!Gm->POSITION[c][6]) Gm->Matsig&=~(0x001<<(4+5*c));				// no pawn left? remove from matsig		
   (Gm->Piece[c][t]).type=p; 	(Gm->Piece[c][t]).index=i;				// type and index of new piece
   (Gm->Officer[c][i]).type=p;	(Gm->Officer[c][i]).square=t;			// type and square of new piece
   (Gm->Psv[c]).Open+=Square[c][p-1][0][t]-Square[c][5][0][t];			// piece square value opening
   (Gm->Psv[c]).End +=Square[c][p-1][1][t]-Square[c][5][1][t];			// piece square value endgame
   HB^=RANDOM_B[c][5][t];												// remove pawn from position hash
   Gm->PHASH^=RANDOM_B[c][5][t];										// remove pawn from pawn evaluation hash
   Gm->MHASH-=RANDOM_B[c][5][0];										// remove pawn from material hash
   HB^=RANDOM_B[c][p-1][t];												// set new piece in position hash
   Gm->MHASH+=RANDOM_B[c][p-1][0];										// set new piece in material hash
  }
  fi=0;																	// reset fifty move counter	
 }
 else (Gm->Officer[c][o]).square=t;										// set new piece square
 if(cas&0x0F)															// deal with castle (-rights)
 {
  if(cas&1) if((f==60)||(f==63)||(t==63)) {cas&=254; HB^=RANDOM_P[8];}	// white castles kingside lost
  if(cas&2) if((f==60)||(f==56)||(t==56)) {cas&=253; HB^=RANDOM_P[9];}	// white castles queenside lost	  
  if(cas&4) if((f==4)||(f==7)||(t==7))    {cas&=251; HB^=RANDOM_P[10];}	// black castles kingside lost
  if(cas&8) if((f==4)||(f==0)||(t==0))	  {cas&=247; HB^=RANDOM_P[11];}	// black castles queenside lost
  if((p==1)&&((t-f==2)||(f-t==2)))										// castles
  {
   if(c) i=0x40; else i=0x10;											// castles black or white
   if(t-f==2) {f+=3; t--;} else {f-=4; t++; i<<=1;}						// king- or queenside
   (Gm->Psv[c]).Open+=Square[c][2][0][t]-Square[c][2][0][f];			// piece square value rook opening
   (Gm->Psv[c]).End +=Square[c][2][1][t]-Square[c][2][1][f];			// piece square value rook endgame
   FB=A8<<f; TB=A8<<t; cas|=i;											// bitmaps of move     
   (Gm->Piece[c][t]).type=3;											// move rook to destination square
   i=(Gm->Piece[c][t]).index=(Gm->Piece[c][f]).index;					// move index to destination
   (Gm->Piece[c][f]).type 	=(Gm->Piece[c][f]).index=0;					// clear rook from origin square						
   (Gm->Officer[c][i]).square=t;										// set new rook square
   Gm->POSITION[c][0]^=TB|FB; Gm->POSITION[c][3]^=TB|FB;				// move rook in type-bitmaps		
   Gm->ROTATED[0]-=FB; Gm->ROTATED[0]|=TB;								// move rook in general bitmap								 
   for(j=1;j<4;j++)														// move rook in rotated bitmaps
   {
    s=shift[j+3][f]; Gm->ROTATED[j]-=s<128?FB<<s:FB>>(256-s);
    s=shift[j+3][t]; Gm->ROTATED[j]|=s<128?TB<<s:TB>>(256-s);  
   }  
   HB^=RANDOM_B[c][2][f]; HB^=RANDOM_B[c][2][t];						// update hash for moving rook
  }
 }
 Finish:
 (Gm->Moves[Gm->Move_n]).HASH=HB; 	  (Gm->Moves[Gm->Move_n]).fifty=fi;	// store data of position after move
 (Gm->Moves[Gm->Move_n]).castles=cas; (Gm->Moves[Gm->Move_n]).ep=ep;
 return;
}

void 	UnMove(Game* Gm)												// takes back move
{
 Byte 	c,i,j,k,f,t,s,p,o,pc;
 BitMap	CB,FB,TB;
 
 c=1-Gm->color; Gm->color=c; (Gm->Move_n)--; 							// restore color and move number
 f=(Gm->Moves[Gm->Move_n]).from;  										// get origin
 t=(Gm->Moves[Gm->Move_n]).to;											// get destination
 if(f==t) return;														// nullmove
 FB=A8<<f; TB=A8<<t;													// from and to bitmaps
 p=(Gm->Piece[c][t]).type; o=(Gm->Piece[c][t]).index;					// type and index of piece
 (Gm->Psv[c]).Open+=Square[c][p-1][0][f]-Square[c][p-1][0][t];			// piece square value opening
 (Gm->Psv[c]).End +=Square[c][p-1][1][f]-Square[c][p-1][1][t];			// piece square value endgame
 if((Gm->Moves[Gm->Move_n]).prom)										// promotion
 {
  Gm->Matsig|=(1<<(5*c+4));												// set pawn in material signature
  (Gm->Psv[c]).Open+=Square[c][5][0][f]-Square[c][p-1][0][f];			// piece square value opening
  (Gm->Psv[c]).End +=Square[c][5][1][f]-Square[c][p-1][1][f];			// piece square value endgame
  if(p==2) Gm->phase-=4; else if(p==3) Gm->phase-=2; else (Gm->phase)--;// update game phase
  ((Gm->Count[c]).pawns)++; j=--((Gm->Count[c]).officers);				// inc. pawn, dec. officer
  Gm->POSITION[c][6]|=TB; Gm->POSITION[c][p]-=TB;						// set pawn, remove officer
  if(!Gm->POSITION[c][p]) Gm->Matsig&=~(0x001<<(5*c+p-2));				// last piece of this type? remove from matsig
  (Gm->Piece[c][t]).type=6; (Gm->Piece[c][t]).index=0;					// set type as pawn (no index)
  (Gm->Officer[c][j]).type=(Gm->Officer[c][j]).square=0;				// remove officer
  Gm->PHASH^=RANDOM_B[c][5][t];											// introduce pawn in pawn evaluation hash
  Gm->MHASH+=RANDOM_B[c][5][0];											// introduce pawn in material hash
  Gm->MHASH-=RANDOM_B[c][p-1][0];										// subtract piece from material hash							
  p=6;	o=0;															// moving piece type is pawn
 }
 (Gm->Piece[c][f]).type=p; (Gm->Piece[c][f]).index=o;					// set piece on origin square
 (Gm->Piece[c][t]).type=(Gm->Piece[c][t]).index=0;						// clear piece from target square
 Gm->POSITION[c][0]^=TB|FB; Gm->POSITION[c][p]^=TB|FB;					// move piece in type-bitmaps
 if(p<6) (Gm->Officer[c][o]).square=f;									// set officer to origin square
 Gm->ROTATED[0]|=FB;													// set piece in general bitmaps											
 for(j=1;j<4;j++)														// set piece in rotated bitmaps
  {s=shift[j+3][f]; Gm->ROTATED[j]|=s<128?FB<<s:FB>>(256-s);} 
 if((p==6)||(p==1))														// pawn or king move
  {Gm->PHASH^=RANDOM_B[c][p-1][t]; Gm->PHASH^=RANDOM_B[c][p-1][f];}		// update pawn evaluation hash
 if(s=(Gm->Moves[Gm->Move_n]).cap)										// capture
 {
  if(s&8)																//ep	
  {
   i=t+8-16*c; CB=(A8<<i); Gm->ROTATED[0]^=CB|TB;						// set ep-pawn in general bitmap 
   for(j=1;j<4;j++) 													// remove pawn in rotated bitmaps
    {o=shift[j+3][t]; Gm->ROTATED[j]-=o<128?TB<<o:TB>>(256-o);}
   for(j=1;j<4;j++)														// set ep-pawn in rotated bitmaps
    {o=shift[j+3][i]; Gm->ROTATED[j]|=o<128?CB<<o:CB>>(256-o);} 
  }	
  else {i=t; CB=(A8<<t);}												// ordinary capture
  pc=s&7; o=s>>4;														// captured piece type and index
  (Gm->Piece[1-c][i]).type=pc; (Gm->Piece[1-c][i]).index=o;				// type and index of captured piece
  Gm->POSITION[1-c][pc]|=CB; Gm->POSITION[1-c][0]|=CB; 					// set piece in type-bitmap
  Gm->Matsig|=(0x001<<(3+pc-5*c));										// set piece in material signature
  (Gm->Psv[1-c]).Open+=Square[1-c][pc-1][0][i];							// piece square value opening
  (Gm->Psv[1-c]).End +=Square[1-c][pc-1][1][i];							// piece square value endgame
  Gm->MHASH+=RANDOM_B[1-c][pc-1][0];									// add piece to material hash
  if(pc==6) 															// increase pawn counter
   {((Gm->Count[1-c]).pawns)++; Gm->PHASH^=RANDOM_B[1-c][5][i];}		// increase pawn counter and create pawn...	
  else																	// ...in pawn evaluation hash		
  {  
   if(pc==2) Gm->phase+=4; else if(pc==3) Gm->phase+=2; else (Gm->phase)++;// update game phase
   j=((Gm->Count[1-c]).officers)++;										// increase officer counter
   if(j!=o)
   {
    (Gm->Officer[1-c][j]).type=(Gm->Officer[1-c][o]).type;				// free space in list
    k=(Gm->Officer[1-c][j]).square=(Gm->Officer[1-c][o]).square;
    (Gm->Piece[1-c][k]).index=j;
   }
   (Gm->Officer[1-c][o]).type=pc; (Gm->Officer[1-c][o]).square=i;  		// include piece in list	
  }
 }
 if(!s)																	// no capture
 {
  Gm->ROTATED[0]-=TB;													// clear piece from bitmap
  for(j=1;j<4;j++)														// remove piece in rotated bitmaps
   {s=shift[j+3][t]; Gm->ROTATED[j]-=s<128?TB<<s:TB>>(256-s);}
 }
 if((p==1)&&((t-f==2)||(f-t==2)))										// castles
 {
  if(t-f==2) {f+=3; t--;}							 					// castles kingside
  else {f-=4; t++;}														// castles queenside
  FB=(A8<<f); TB=(A8<<t);												// bitmaps of move
  (Gm->Piece[c][f]).type=3;												// move rook to origin square
  j=(Gm->Piece[c][f]).index=(Gm->Piece[c][t]).index;					// move index to origin
  (Gm->Piece[c][t]).type=(Gm->Piece[c][t]).index=0;						// clear rook from destination square
  (Gm->Officer[c][j]).square=f;											// set rook origin square
  (Gm->Psv[c]).Open+=Square[c][2][0][f]-Square[c][2][0][t];				// piece square value opening
  (Gm->Psv[c]).End +=Square[c][2][1][f]-Square[c][2][1][t];				// piece square value endgame				
  Gm->POSITION[c][0]^=TB|FB;	Gm->POSITION[c][3]^=TB|FB;				// move rook in type-bitmaps					
  Gm->ROTATED[0]-=TB; Gm->ROTATED[0]|=FB;								// move rook in general bitmaps 							
  for(j=1;j<4;j++)														// move rook in rotated bitmaps
  {
   s=shift[j+3][t]; Gm->ROTATED[j]-=s<128?TB<<s:TB>>(256-s);
   s=shift[j+3][f]; Gm->ROTATED[j]|=s<128?FB<<s:FB>>(256-s);  
  }
 }
 return;
}

void 	PrintBM(BitMap BM)												// prints bitmap
{
 Byte i;

 for(i=0;i<64;)
  {
   printf("%llu ",(BM>>i)&1);
   if(!(++i&7)) printf("\n");
  }
 printf(".\n");
}

void 	Divide(Game* Gm, Byte d)										// calculates divide
{
 Game		Gam[Paras[93].Val];											// database for threads
 Mvs  		Mv,Mov[Paras[93].Val];
 BitMap		NOD;
 Dbyte  	m;
 Byte		i,j,t;
 char 		Mo[10];
 clock_t 	t1;
 double		t2;
 
 for(NOD=0;NOD<DEN;NOD++) for(i=0;i<16;i++) *(hash_t+NOD*16+i)=0;		// clear hash table															
 t1=clock();															// start stopwatch
 printf("Using %d thread(s) to calculate divide(%d)\n\n",Paras[93].Val,d);
 if(!d) {printf("0 leafs\n"); return;}									// trivial case	
 GenMoves(Gm,&Mv); if(!(Mv.cp&63)) {printf("0 leafs"); return;}			// generate moves, no moves
 NOD=0;	Gm->idepth=d-1; Gm->Finished=true; Gm->Currm=0; 				// initialize node count
 Mv.o=0; Mv.s=200; Mv.flg=0;											// init move picker
 for(i=0;i<Paras[93].Val;i++) {Mov[i]=Mv; Gam[i]=*Gm;}					// replicate thread databases
 i=j=t=0;
 do for(j=0;j<Paras[93].Val;j++)										// scan threads 
 {
  if(Gam[j].Finished&&(Gam[j].idepth==d-1)&&Gam[j].Currm)				// thread finished a move
  {
   pthread_join(Gam[j].Tid,NULL);										// wait for thread and translate move
   i++; UncodeMove(Gam[j].Currm,Mo);									// translate move 
   printf("Thread %d: %2d. %s: %llu leafs\n",j+1,i,Mo,Gam[j].NODES);	// print thread, move and node count
   NOD+=Gam[j].NODES; Gam[j].Currm=0; UnMove(Gam+j); t--;				// add nodes and unmake move
  } 
  if(Gam[j].Finished&&(Gam[j].idepth==d-1)&&(!(Gam[j].Currm)))			// free thread found
   if(m=PickMove(Gm,&Mv))												// next move
   { 
    //pthread_join(Gam[j].Tid,NULL); 									// wait for thread
	UncodeMove(m,Mo); 													// translate move
	printf("Thread %d calculates move %s\n",j+1,Mo);					// inform about thread start
    t++; Move(Gam+j,m); Gam[j].Finished=false; Gam[j].Currm=m;			// prepare thread with move
    pthread_create(&Gam[j].Tid, NULL, PerftCount, (void*)(Gam+j));		// start thread
   }
 } 
 while(t);																// threads still running
 t2=(double)(clock()-t1)/CLOCKS_PER_SEC;								// stop stopwatch
 printf("\n%d moves, %llu leafs of %d-ply tree\n",i,NOD,d);				// print result
 printf("(%.3lfs, %.0lf leafs/s)\n",t2,(double)(NOD/t2));				// print time	
}

void 	Perft(Game* Gm, Byte d)											// calculates perft
{
 Game		Gam[Paras[93].Val];											// database for threads
 Mvs  		Mv,Mov[Paras[93].Val];
 BitMap		NOD;
 Dbyte  	m;
 Byte		i,j,t;
 clock_t 	t1;
 double		t2;
 
 for(NOD=0;NOD<DEN;NOD++) for(i=0;i<16;i++) *(hash_t+NOD*16+i)=0;		// clear hash table														
 t1=clock();															// start stopwatch
 printf("Using %d thread(s) to calculate perft(%d):\n",Paras[93].Val,d);
 if(!d) {printf("0 leafs\n"); return;}									// trivial case
 GenMoves(Gm,&Mv); if(!(Mv.cp&63)) {printf("0 leafs"); return;}			// generate moves, no moves
 NOD=0;	Gm->idepth=d-1; Gm->Finished=true; Gm->Currm=0;  				// initialize node count
 Mv.o=0; Mv.s=200; Mv.flg=0;											// init move picker
 for(i=0;i<Paras[93].Val;i++) {Mov[i]=Mv; Gam[i]=*Gm;}					// replicate thread databases
 i=j=t=0;
 do for(j=0;j<Paras[93].Val;j++)										// scan threads 
 {
  if(Gam[j].Finished&&(Gam[j].idepth==d-1)&&Gam[j].Currm)				// thread finished a move
  {
   pthread_join(Gam[j].Tid,NULL);										// wait for thread and translate move
   i++; NOD+=Gam[j].NODES; Gam[j].Currm=0; UnMove(Gam+j); t--;			// add nodes and unmake move 
  } 
  if(Gam[j].Finished&&(Gam[j].idepth==d-1)&&(!Gam[j].Currm))			// free thread found
   if(m=PickMove(Gm,&Mv))												// next move
   { 
    
    t++; Move(Gam+j,m); Gam[j].Finished=false; Gam[j].Currm=m;			// prepare thread with move
    pthread_create(&Gam[j].Tid, NULL, PerftCount, (void*)(Gam+j));		// start thread
   }
 } 
 while(t);																// threads still running
 t2=(double)(clock()-t1)/CLOCKS_PER_SEC;								// stop stopwatch
 printf("\n%d moves, %llu leafs of %d-ply tree\n",i,NOD,d);				// print result
 printf("(%.3lfs, %.0lf leafs/s)\n",t2,(double)(NOD/t2));				// print time
}

void 	*PerftCount(void *Pe)											// recursive perft
{
 Game	*Gm;
 Byte	i;
 Mvs	Mv;
 BitMap NOD,PM;
 Dbyte	m;
 
 Gm=(Game*)(Pe); Gm->Finished=false;									// initialize run-flag
 if(Gm->idepth==0) {Gm->NODES=1; Gm->Finished=true; return NULL;}		// leaf found, trivial case
 if(((NOD=GetHashP(Gm))!=-1)&&(Gm->idepth==(Byte)(NOD>>56))) 			// check hash info (1 byte depth, 7 bytes data)
  {Gm->NODES=NOD&(0xFFFFFFFFFFFFFF); Gm->Finished=true; return NULL;}	// node count found in hash table
 GenMoves(Gm,&Mv); 														// generate legal moves
 if(!(Mv.cp&63)) {Gm->NODES=0; Gm->Finished=true; return NULL;}			// no legal moves possible
 NOD=0;	Mv.o=0; Mv.s=200; Mv.flg=0;										// init move picker						
 if(Gm->idepth==1) 														// shortcut: count leafs as bits in bitmaps of moves
 {
  for(i=0;i<(Gm->Count[Gm->color]).officers;i++)						// count officer moves by popcount of bitmaps
  	NOD+=Popcount(Mv.OFFM[i]);					
  for(i=0;i<4;i++) 														// count pawn moves by popcount of bitmaps
   {PM=Mv.PAWM[i]; NOD+=Popcount(PM); NOD+=3*Popcount(PM&PROM8);}		// add sub-promotions
 }								
 else 
 {
  Gm->idepth--; 														// reduce depth
  while(m=PickMove(Gm,&Mv))												// next move
   {Move(Gm,m); PerftCount((void*)(Gm)); UnMove(Gm); NOD+=Gm->NODES;}	// recursive node count
  Gm->idepth++;															// increase depth
 }
 StoreHashP(Gm,NOD+((BitMap)(Gm->idepth)<<56));							// store perft data in hash
 Gm->NODES=NOD; Gm->Finished=true; return NULL;							// return node count
}

void 	StoreHashP(Game* Gm, BitMap NOD)								// store perft hash entry
{
 BitMap KEY,LOCK;
 
 LOCK=(Gm->Moves[Gm->Move_n]).HASH; KEY=(LOCK%DEN)*16;					// calculate hash key (modulo entries)
 *(BitMap*)(hash_t+KEY)=NOD; *(BitMap*)(hash_t+KEY+8)=(LOCK^NOD);		// store nodes and lock with consistency check
 return;			
}

BitMap 	GetHashP(Game* Gm)												// get perft hash entry
{
 BitMap KEY,LOCK,NOD,LOCK1;
 
 LOCK=(Gm->Moves[Gm->Move_n]).HASH; KEY=(LOCK%DEN)*16;					// calculate hash key (modulo entries)
 NOD=*(BitMap*)(hash_t+KEY); LOCK1=*(BitMap*)(hash_t+KEY+8); 			// get stored data
 if(LOCK1==(LOCK^NOD)) return NOD;										// consistent entry found
 return -1;																// no entry found or not consistent
}

BitMap 	GetHash(Game* Gm)												// get hash entry
{
 BitMap	LOCK=(Gm->Moves[Gm->Move_n]).HASH;								// lock 
 Byte* 	key=hash_t+(LOCK%HEN)*32;										// key
 BitMap DATA=*(BitMap*)(key);											// contents
 
 if(!(*(BitMap*)(key+8 )^(DATA&NCRC)^LOCK)) return DATA;				// first slot fits with correct crc
 DATA=*(BitMap*)(key+16);
 if(!(*(BitMap*)(key+24)^(DATA&NCRC)^LOCK)) return DATA;				// second slot fits with correct crc
 return 0;																// no fit
}

void	StoreHash(Game* Gm,short Val,Dbyte Move,Byte depth,Byte flags)	// store information in transposition table
{
 BitMap	LOCK=(Gm->Moves[Gm->Move_n]).HASH;								// lock 
 Byte* 	key=hash_t+(LOCK%HEN)*32;										// key
 BitMap DATA11=*(BitMap*)(key);											// contents slot 1
 BitMap DATA12=*(BitMap*)(key+8);
 BitMap DATA21=*(BitMap*)(key+16);										// contents slot 2
 BitMap DATA22=*(BitMap*)(key+24);
 
 if(DATA11&0x0400)	HASHFILL++;											// first entry
 else if((DATA12^(DATA11&NCRC)^LOCK)&&((!(DATA22^(DATA21&NCRC)^LOCK))||	// other position at first slot and same position 
 		 (DATA21&0x0800)||((Byte)(DATA21&0xFF)<=(Byte)(DATA11&0xFF))))	// at second slot or second slot is old	or has lower 					 
  {*(BitMap*)(key+16)=DATA11; *(BitMap*)(key+24)=DATA12;}				// draft than 1st copy first slot to second slot
  
 if(Val<(short)(255-MaxScore)) 		Val-=(short)(Gm->Move_n-Gm->Move_r);// adjust negative mate score
 else if(Val>(short)(MaxScore-255)) Val+=(short)(Gm->Move_n-Gm->Move_r);// adjust positive mate score

 DATA11=((BitMap)(Move)<<32)+(((BitMap)(Val)&0xFFFF)<<16)+				// construct entry
 								((BitMap)(flags)<<8)+(BitMap)(depth);	
 
 *(BitMap*)(key)=DATA11; *(BitMap*)(key+8)=(DATA11&NCRC)^LOCK;			// save transposition info with crc

}

bool 	DrawTest(Game* Gm)												// position is a draw?
{
 Dbyte	Mm,Mn=Gm->Move_n;												// move number
 Dbyte	Fi=(Dbyte)((Gm->Moves[Mn]).fifty);								// fifty moves counter
 BitMap	HM=(Gm->Moves[Mn]).HASH;										// current hash value
 int	Mo;
 
 if((Fi<4)||(Mn<3)) 									return false;	// no draw possible
 if(Fi>99) 												return true;	// fifty move rule

 if(Mn>Fi) Mm=Mn-Fi; else Mm=0;											// only reversible moves must be checked
 for(Mo=Mn-4;Mo>=Mm;Mo-=2) if(HM==(Gm->Moves[(Dbyte)(Mo)]).HASH) return true;	// same position existed before!
 return false;															// no draw
}

short 	Popcount(BitMap BM)												// counts set bits
{
 BM = BM - ((BM >> 1) & 0x5555555555555555);        					// add pairs of bits
 BM = (BM & 0x3333333333333333) + ((BM >> 2) & 0x3333333333333333);  	// quads
 BM = (BM + (BM >> 4)) & 0x0F0F0F0F0F0F0F0F;        					// groups of 8
 return (BM * 0x0101010101010101) >> 56;         						// horizontal sum of bytes
}

Dbyte Book(Game* Gm)
{
 int	i;
 Byte	r,k,t,f;
 Dbyte	Mov,Mov2,Om[100];
 Game	GmTest;
 Mvs	Mv;
 BitMap	BM;
 

 if(!Options[2].Val)									goto Tree;		// no position library
 k=0; Mov=0; for(i=0;i<sizeof(AOB_POS)/8;i++)							// search position in opening book
  if(!((AOB_POS[i]^(Gm->Moves[Gm->Move_n]).HASH)>>15)) k++;				// k moves found
 if(!k)													goto Tree;		// no move found
 r=rand()%k; k=0;
 for(i=0;i<sizeof(AOB_POS)/8;i++)										// search position in opening book
  if((!((AOB_POS[i]^(Gm->Moves[Gm->Move_n]).HASH)>>15))&&(k++==r))		// randomly selected position found
   {Mov=(Dbyte)(AOB_POS[i]&0x7FFF); break;}								// get the move
 if(Mov)																// check if move is legal
 {
  GenMoves(Gm,&Mv); if(!Mv.cp)							return(0);		// no move possible
  t=56-(Mov&56)+(Mov&7); Mov>>=6; f=56-(Mov&56)+(Mov&7);				// extract origin and destination
  if(!(r=(Gm->Piece[Gm->color][f]).type))				goto Tree;		// piece on origin does not exist
  if(r==1) if(f==60) 	 {if(t==63) t=62; else if(t==56) t=58;}			// correct castles target
  		   else if(f==4) {if(t==7)  t=6;  else if(!t)    t=2;}
  BM=(A8<<t); 
  if(r<6) {if(!(Mv.OFFM[(Gm->Piece[Gm->color][f]).index]&BM)) goto Tree;}//officer move not possible
  else if((t-f==16)||(f-t==16)) {if(!(BM&Mv.PAWM[3])) goto Tree;}		// double step pawn move not possible
  else {r=9-16*Gm->color+t-f; if((r>2)||(!(BM&Mv.PAWM[r]))) goto Tree;}	// pawn move not possible
  Mov2=(t<<8)+f; if(r=(Mov>>6)) Mov2+=((4-r)<<14)+128; return(Mov2);	// translate move from polyglot format													
 }
 
 Tree:
 if(!Options[1].Val)									return(0);		// no position library
 GetPosition(&GmTest,Startpos);											// start from initial position
 i=0; k=0; Mov=0; BM=(Gm->Moves[Gm->Move_n]).HASH;							
 while(Mov=Aob_tre[i++])												// parse database
  if((f=(Mov&0xFF))==(Mov>>8)) for(t=0;t<f;t++) UnMove(&GmTest);		// end of opening line, take back f moves
  else
  {
   if(BM==(GmTest.Moves[GmTest.Move_n]).HASH) if(!(Mov&64)) Om[k++]=Mov;// Position found, record move
   Move(&GmTest,Mov&0xFFBF);											// make move
  }
 if(k) 											return(Om[rand()%k]);	// pick random move
 												return(0);				// position not found
}

bool 	SEE(Game *Gm, Dbyte Mov, short Thr)								// SEE of move larger or equal threshold?
{
 short	Val;
 Byte 	f,t,p,s,pm,a,pn;
 BitMap BM,BO[4],OM;

 if(Mov&128) return true;												// promotion
 t=(Byte)(Mov>>8)&63; f=(Byte)(Mov)&63;									// origin and destination
 if(p=(Gm->Piece[1-Gm->color][t]).type) Val=Paras[3+p].Val; 			// possible SEE gain is value of piece on destination
 else Val=0;
 if(Val<Thr) return false;												// value of piece to gain is lower than threshold
 BM=(A8<<f); BO[0]=(Gm->ROTATED[0]&ATC[0][t])&(~(BM|(A8<<t)));			// potential attackers of destination square
 if(!(BO[0]&(Gm->POSITION[1-Gm->color][0]))) return true;				// no attackers: gain is piece value
 for(p=1;p<4;p++) 
 {
  s=shift[p+3][f]; 
  BO[p]=Gm->ROTATED[p]&~(s<128?BM<<s:BM>>(256-s));						// remove moving piece from rotated bitmaps
 }
 pm=(Gm->Piece[Gm->color][f]).type;										// moving piece type
 while(BO[0])															// parse potential attackers
 {
  if(Val-Paras[3+pm].Val>=Thr) return true;								// even if piece is lost, move is worthwhile
  a=0; pn=7; while((!a)&&(--pn))										// parse potential attackers 
  if(OM=(Gm->POSITION[1-Gm->color][pn]&ATC[pn][t]&BO[0])) switch(pn)	// potential attacker exists
  {
   case 2: if(BM=(SLIDE[t][0][BO[0]>>shift[0][t]&255]&OM))
   			{a=1; break;}
		   if(BM=(SLIDE[t][1][BO[1]>>shift[1][t]&255]&OM))	
   	        {a=1; break;}												// queen is real attacker
   case 4: if(BM=(SLIDE[t][2][BO[2]>>shift[2][t]&255]&OM))
   			{a=1; break;}
   		   if(BM=(SLIDE[t][3][BO[3]>>shift[3][t]&255]&OM))	
   	        a=1; break;													// bishop or queen is real attacker   	
   case 3: if(BM=(SLIDE[t][0][BO[0]>>shift[0][t]&255]&OM))
   			{a=1; break;}
   		   if(BM=(SLIDE[t][1][BO[1]>>shift[1][t]&255]&OM))
   			a=1; break;													// rook is real attacker
   case 6: if(BM=(STEP[3+Gm->color][t]&OM)) a=1; break;					// pawn is real attacker
   default: BM=OM; a=1;													// king or knight is real attacker
  }
  if(!a) return (Val>=Thr);												// no more attackers: value is definite
  Val-=Paras[3+pm].Val; 												// subtract attacked piece value, 
  if(Val+Paras[3+pn].Val<Thr) return false;								// even if attacker is captured value is below threshold
  BM&=-BM; BO[0]&=~BM; f=find_b[(BM^BM-1)%67];							// pick one attacker and remove, square of attacker
  for(p=1;p<4;p++)
  {
   s=shift[p+3][f]; BO[p]&=~(s<128?BM<<s:BM>>(256-s));					// remove attacker from rotated bitmaps
  }
  a=0; pm=7; while((!a)&&(--pm))										// parse potential defenders
  if(OM=(Gm->POSITION[Gm->color][pm]&ATC[pm][t]&BO[0])) switch(pm)		// potential defender exists 
  {
   case 2: if(BM=(SLIDE[t][0][BO[0]>>shift[0][t]&255]&OM))
   			{a=1; break;}
		   if(BM=(SLIDE[t][1][BO[1]>>shift[1][t]&255]&OM))	
   	        {a=1; break;}												// queen is real defender
   case 4: if(BM=(SLIDE[t][2][BO[2]>>shift[2][t]&255]&OM))
   			{a=1; break;}
   		   if(BM=(SLIDE[t][3][BO[3]>>shift[3][t]&255]&OM))	
   	        a=1; break;													// bishop or queen is real defender   	
   case 3: if(BM=(SLIDE[t][0][BO[0]>>shift[0][t]&255]&OM))
   			{a=1; break;}
   		   if(BM=(SLIDE[t][1][BO[1]>>shift[1][t]&255]&OM))
   			a=1; break;													// rook is real defender
   case 6: if(BM=(STEP[4-Gm->color][t]&OM)) a=1; break;					// pawn is real defender
   default: BM=OM; a=1;													// king or knight is real defender
  }
    
  if(!a) return (Val>=Thr);												// no more defenders: value is definite
  Val+=Paras[3+pn].Val;													// add defenders piece value
  BM&=-BM; BO[0]&=~BM; f=find_b[(BM^BM-1)%67];							// pick one defender and remove, square of defender
  for(p=1;p<4;p++)
   {s=shift[p+3][f]; BO[p]&=~(s<128?BM<<s:BM>>(256-s));}				// remove defender from rotated bitmaps
 }
 return false;
}

short 	MatEval(Game* Gm)												// material+pst+stm evaluation
{
 BitMap	LOCK,PM,QM;
 short	Pa,Pw,Pb,Val,j;
 Byte	i;

 LOCK=*(BitMap*)(mhash_t+((Gm->MHASH)%MEN)*8); Val=(short)(LOCK);		// material hash entry
 if((short)((LOCK^Gm->MHASH)>>16)!=Val)									// entry does not fit or crc wrong
 {
  Pw= Pb=0;																// initialize piece value
  for(i=1;i<(Gm->Count[0]).officers;i++) 
   Pw+=Paras[3+(Gm->Officer[0][i]).type].Val; 							// accumulate piece values white
  for(i=1;i<(Gm->Count[1]).officers;i++) 
   Pb+=Paras[3+(Gm->Officer[1][i]).type].Val; 							// accumulate piece values black	

  Val=Pw-Pb+(short)((Gm->Count[0]).pawns-(Gm->Count[1]).pawns)*			// add pawn values
  												Paras[9].Val;

  PM=Gm->POSITION[0][4]; if((PM&WS)&&(PM&BS)) Val+=Paras[10].Val; 		// pair of bishops bonus white 	
  QM=Gm->POSITION[1][4]; if((QM&WS)&&(QM&BS)) Val-=Paras[10].Val; 		// pair of bishops bonus black
  if((!(PM&(PM-1)))&&(!(QM&(QM-1)))&&((PM|QM)&WS)&&((PM|QM)&BS)) i=1; 	// single differently colored bishops
  else i=0;	
  
  Pa=Paras[9].Val*((short)((Gm->Count[0]).pawns)-5);					// value of number of pawns above 5 for white	
  PM=Gm->POSITION[0][5]; while(PM) {Val+=Pa/16; PM&=PM-1;}				// bonus for knights with pawns on the board 								
  PM=Gm->POSITION[0][3]; while(PM) {Val-=Pa/8;  PM&=PM-1;}				// malus for rooks with pawns on the board					
  Pa=Paras[9].Val*((short)((Gm->Count[1]).pawns)-5);					// value of number of pawns above 5 for white	
  PM=Gm->POSITION[1][5]; while(PM) {Val-=Pa/16; PM&=PM-1;}				// bonus for knights with pawns on the board 								
  PM=Gm->POSITION[1][3]; while(PM) {Val+=Pa/8;  PM&=PM-1;}				// malus for rooks with pawns on the board

  j=64;
  
  if(Val>0)																// white is leading
  {
   if((!((Gm->Count[0]).pawns))&&(Pw-Pb<=Paras[7].Val))					// white has no pawns and is less than a bishop ahead
    j=(Pw<Paras[6].Val) ? 0 : ((Pb<=Paras[7].Val) ? 4 : 14);			// scale factor for reduced material
   else if(i&&((Gm->Count[0]).officers==2)&&
   				((Gm->Count[1]).officers==2)) j=22;						// opposing bishops
   else j=min(64,36+(i?2:7)*(Gm->Count[0]).pawns);						// scale factor for reduced material
  }
  else if(Val<0)														// black is ahead
  {
   if((!((Gm->Count[1]).pawns))&&(Pb-Pw<=Paras[7].Val))					// black has no pawns and is less than a bishop ahead
    j=(Pb<Paras[6].Val) ? 0 : ((Pw<=Paras[7].Val) ? 4 : 14);			// scale factor for reduced material
   else if(i&&((Gm->Count[0]).officers==2)&&
   				((Gm->Count[1]).officers==2)) j=22;						// opposing bishops
   else j=min(64,36+(i?2:7)*(Gm->Count[1]).pawns);						// scale factor for reduced material
  }
  
  Val=(Val*(short)(j))/64;												// scaling the material value
  LOCK=Val+(((Gm->MHASH>>16)^Val)<<16);									// calculate entry with Val as crc
  *(BitMap*)(mhash_t+((Gm->MHASH)%MEN)*8)=LOCK;							// save entry
 }

 j=min(Gm->phase,24);													// game phase
 
 Val+=(((((Gm->Psv[0]).Open-(Gm->Psv[1]).Open)*j+						// tempered piece square value for opening...
 	  ((Gm->Psv[0]).End-(Gm->Psv[1]).End)*(24-j))/48)*					// ... and endgame;
	   Paras[11].Val)/10;												 
	   	   	   	  
 if(Gm->color) Val=-Val;												// stm is black 
  	  
 return Paras[12].Val+Val;												// stm bonus
}

short	Evaluation(Game* Gm, Mvs* Mv, short Alpha, short Beta)			// evaluation
{
 BitMap ELOCK,EHASH,PLOCK[5],PP[2],AP[2],PAI,PAB,PAD,PAP,PAC,PAW,PCH,PPH,CM;
 BitMap	KM,PS,NM,MM,MP[5],PM,PN,PO;
 short	Val,Oval,Eval,Dval,cl;
 Byte	pk[2],gp,flg,co,sp,p;
 Byte	*ekey,*pkey;
 
 //return MatEval(Gm);

 EHASH=(Gm->Moves[Gm->Move_n]).HASH; ekey=ehash_t+(EHASH%EEN)*8;		// evaluation hash table entry address
 ELOCK=*(BitMap*)(ekey); Val=(short)(ELOCK);							// evaluation is in the first two bytes
 if(((ELOCK^EHASH)>>16)==(BitMap)(Val)&0xFFFF)		return Val;			// evaluation found in hash table
 
 gp=min(24,Gm->phase);													// game phase 0 (endgame) to 24 (opening)
 Val=MatEval(Gm);														// material evaluation
 PP[0]=Gm->POSITION[0][6]; PP[1]=Gm->POSITION[1][6];					// pawn positions
 pk[0]=(Gm->Officer[0][0]).square; pk[1]=(Gm->Officer[1][0]).square;	// pawn positions
 
 pkey=phash_t+(Gm->PHASH%PEN)*40; 										// pawn hash key and weak pawns
 PLOCK[0]=*(BitMap*)(pkey); 	  PLOCK[1]=*(BitMap*)(pkey+8); 			// lock 0..3
 PLOCK[2]=*(BitMap*)(pkey+16); 	  PLOCK[3]=*(BitMap*)(pkey+24); 		// 
 PLOCK[4]=*(BitMap*)(pkey+32);
 
 if(Gm->PHASH^PLOCK[0]^PLOCK[1]^PLOCK[2]^PLOCK[3]^PLOCK[4])				// entry or crc wrong
 {
  Oval=Eval=0; PAI=PAB=PAD=PAP=PAC=PAW=PCH=PPH=0;	flg=0;				// intialize values and pawn bitmaps
	
  for(CM=H;CM>A1;CM>>=1) for(co=0;co<2;co++)							// scan files H to A and colors
  {
   KM=CM&PP[co]; if(KM&(KM-1)) 							PAD|=KM;		// multiple pawns -> register in double pawn bitmap
   while(KM)															// all pawns on this file
   {
    sp=find_b[(KM^KM-1)%67]; PS=KM&(-KM); KM-=PS; NM=0;   				// sp=pawn square, ps=bitmap of this pawn, remove from file									
    
	if((!(PAWN_E[co][1][sp]&PP[1-co]))&&								// check front and side spawns
       (!(PAWN_E[co][0][sp]&PP[co]))) 					PAP|=PS;		// register passed pawn (no double passed pawn!)
    if( !(PAWN_E[co][4][sp]&PP[co])) 	 				PAI|=PS;		// register isolated pawn 
    if(!((MM=PAWN_E[0][7][sp])&PP[co])) while(MM)						// test for weak pawn. no neighbouring pawns of same color
    {
     if(co) MM>>=8; else MM<<=8;										// next row
     if(!MM) break;														// off the board
     if(MM&PP[co]) {NM=0; break;}										// pawn of same color									
	 if(MM&PP[1-co]) {NM=PS; MM-=(MM&PP[1-co]);}						// pawn of opposite color
	}													PAW|=NM;		// register weak pawn
    if((!(PAWN_E[co][3][sp]&PP[co]))&&									// no friendly pawns on neighbour files and
	   (!(PAWN_E[co][0][sp]&(PP[co]|PP[1-co])))&&						// no pawns in front			
       (PP[1-co]&STEP[3+co][sp-8+16*co])) 				PAB|=PS;		// front square controlled by enemy pawn -> register backward pawn
    MM=PAWN_E[co][6][sp]&PP[1-co]; NM=PAWN_E[co][3][sp]&PP[co];			// sentries and helpers
    if((!(PAWN_E[co][0][sp]&PP[1-co]))&&MM&&NM&&						// no blocking pawn and at least one sentry and helper
      ((!(MM&(MM-1))||(NM&(NM-1)))))  					PAC|=PS;		// at most 1 sentry or two helpers: candidate
   }
  }
  PAW&=~PAI;															// isolated pawns already get malus
  
  PCH|=PP[0]&(((PP[0]&PH)>>7)|((PP[0]&PA)>>9));							// pawn chains white
  PCH|=PP[1]&(((PP[1]&PH)<<9)|((PP[1]&PA)<<7));							// pawn chains black
  PPH|=PP[0]&(((PP[0]&PH)<<1)|((PP[0]&PA)>>1));							// pawn phalanx white
  PPH|=PP[1]&(((PP[1]&PH)<<1)|((PP[1]&PA)>>1));							// pawn phalanx black
  
  // pawn evaluation
 
  for(co=0; co<2; co++)													// evaluate pawn-king positions and pawn chains
  {
   PS=PP[co]; cl=1-2*co; while(PS)										// positions of pawns
   {
    sp=find_b[(PS^PS-1)%67];											// square of pawn
    Eval+=Paras[13].Val*cl*Dist[pk[1-co]][sp];							// distance of opponent king 
    Eval-=Paras[13].Val*cl*Dist[pk[co]][sp];							// distance of friendly king
    PS&=PS-1;															// next pawn
   }
   Dval=Popcount(PCH&PP[co])*Paras[14].Val*cl; Oval+=Dval; Eval+=Dval;	// pawn chains
   Dval=Popcount(PPH&PP[co])*Paras[15].Val*cl; Oval+=Dval; Eval+=Dval;	// pawn phalanx
  }
  for(co=0;co<2;co++)													// evaluate pawn structure
  {
   cl=1-2*co; 
   PS=PP[co]&PAD;														// double pawns 
   while(PS) {Oval-=Paras[16].Val*cl; Eval-=Paras[17].Val*cl; PS&=PS-1;}// opening and endgame malus
   PS=PP[co]&PAI;														// isolated pawns
   while(PS) {Oval-=Paras[18].Val*cl; Eval-=Paras[19].Val*cl; PS&=PS-1;}// opening and endgame malus
   PS=PP[co]&PAB;														// backward pawns
   while(PS) {Oval-=Paras[20].Val*cl; Eval-=Paras[21].Val*cl; PS&=PS-1;}// opening and endgame malus
   PS=PP[co]&PAW;														// weak pawns
   while(PS) {Oval-=Paras[22].Val*cl; Eval-=Paras[23].Val*cl; PS&=PS-1;}// opening and endgame malus
 
   CM=PS=PP[co]&PAP;													// passed pawns
    
   while(PS)															// parse passed pawns
   {
    sp=find_b[(PS^PS-1)%67];											// square of pawn
    Eval+=Paras[24].Val*cl*(2*Dist[pk[1-co]][sp]-Dist[pk[co]][sp]);		// distance to kings
    if(co) Dval=(sp>>3); else Dval=7-(sp>>3); Dval*=Dval; 				// rank of passed pawn
	Oval+=Paras[25].Val*Dval*cl; Eval+=Paras[26].Val*Dval*cl;			// general opening and endgame bonus			
    if(PAWN_E[co][4][sp]&CM) Eval+=(Dval*cl)/Paras[27].Val;				// connected passed pawn
	PS&=PS-1;															// next pawn
   }

   PS=PP[co]&PAC;														// candidate pawns
   while(PS)
   {
    sp=find_b[(PS^PS-1)%67];											// square of pawn
    Eval+=Paras[28].Val*cl*(2*Dist[pk[1-co]][sp]-Dist[pk[co]][sp]);		// distance to kings
    if(co) Dval=(sp>>3); else Dval=7-(sp>>3); Dval*=Dval;				// rank of candidate pawn
	Oval+=Paras[29].Val*Dval*cl; Eval+=Paras[30].Val*Dval*cl;	  		// endgame bonus
    PS&=PS-1;															// next pawn
   }
  }
   
  AP[0]=((PA&PP[0])>>9)|((PH&PP[0])>>7);								// white's pawn attacks
  AP[1]=((PA&PP[1])<<7)|((PH&PP[1])<<9);								// black's pawn attacks
  												
  // king evaluation
  
  Oval-=Paras[31].Val*(3-Popcount(STEP[8][pk[0]]&PP[0]));				// pawn shield white
  Oval+=Paras[31].Val*(3-Popcount(STEP[9][pk[1]]&PP[1]));				// pawn shield black
   
  if(((pk[0]&7)==(pk[1]&7))&&((pk[0]-pk[1]==16)||(pk[1]-pk[0]==16)))	// vertical opposition
  	  															flg|=1;	
  if(((pk[0]>>3)==(pk[1]>>3))&&((pk[0]-pk[1]==2)||(pk[1]-pk[0]==2)))	// horizontal opposition
  	 															flg|=2;
  // store pawn evaluation hash entry
  
  PAW|=(PAI|PAB|PAD);													// weak pawns: isolated,backward,double

  PLOCK[1]=AP[0]|((BitMap)(Oval)<<48); 
  PLOCK[2]=AP[1]|((BitMap)(Eval)&0xFFFF);								// construct pawn hash entry 
  PLOCK[3]=PAP|((BitMap)(flg)&0xFF); PLOCK[4]=PAW;						// save weak pawns						
  *(BitMap*)(pkey)=Gm->PHASH^PLOCK[1]^PLOCK[2]^PLOCK[3]^PLOCK[4];		// save lock with crc
  *(BitMap*)(pkey+8)=PLOCK[1]; *(BitMap*)(pkey+16)=PLOCK[2];			// save hash entry
  *(BitMap*)(pkey+24)=PLOCK[3]; *(BitMap*)(pkey+32)=PLOCK[4];
 }
 else																	// pawn hash entry fits
 {
  Oval=(short)(PLOCK[1]>>48); Eval=(short)(PLOCK[2]&0xFFFF); 			// evaluations and flags
  flg=(Byte)(PLOCK[3]); PAP=PLOCK[3]&0x00FFFFFFFFFFFF00;				// flags and passed pawns
  AP[0]=PLOCK[1]&0x0000FFFFFFFFFFFF; AP[1]=PLOCK[2]&0xFFFFFFFFFFFF0000;	// pawn attacks
  PAW=PLOCK[4]&0x00FFFFFFFFFFFF00;
 }

 // rest of pawn evaluation depending on other pieces
 
 CM=~Gm->POSITION[1-Gm->color][0]; NM=~Gm->POSITION[Gm->color][0];		// squares free of opponent's and stm's pieces
 MP[0]=MP[1]=MP[2]=MP[3]=MP[4]=0;										// initialize stm mvs by piece
 for(co=0;co<(Gm->Count[Gm->color]).officers;co++)						// parse alle stm officers 
  MP[(Gm->Officer[Gm->color][co]).type-1]|=Mv->OFFM[co];				// stm moves by piece

 if(PAP) for(co=0;co<2;co++)											// passed pawns evaluation
 {
  cl=1-2*co; PS=PP[co]&PAP;												// passed pawns of color 
  if(co==Gm->color) {KM=Mv->OATK[0]&CM; MM=Mv->OATK[4]&CM; p=16*co-8;} 	// opponent's moves, bishop moves ...
  else {KM=(Mv->ADES|AP[Gm->color])&NM; MM=MP[3]; p=0;}					// stm correction for Berger
  while(PS)																// parse passed pawns
  {
   sp=find_b[(PS^PS-1)%67];	PM=A8<<(sp+16*co-8);						// square and stop square of pawn
   if(co) Dval=(sp>>3); else Dval=7-(sp>>3);							// rank of passed pawn
   if(PAWN_E[co][0][sp]&Gm->ROTATED[0])									// piece in front
    {Eval-=Paras[32].Val*cl; if(PM&Gm->ROTATED[0]) Eval-=Paras[33].Val*cl;}	// malus for stop/telestop 
   else if(!(PAWN_E[co][2][sp+p]&Gm->POSITION[1-co][1]))				// opponent king not in Berger's square
   	if((Gm->Count[1-co]).officers<2)	Eval+=Dval*Paras[34].Val*cl;	// race bonus
	else if(((Gm->Count[1-co]).officers<3)&&(!(PAWN_E[co][0][sp]&KM))) 		
										Eval+=Paras[35].Val*cl;			// bonus for free path outside berger
   if(PAWN_E[co][0][sp]&KM) 			Eval-=Paras[36].Val*cl;			// malus for stop-square attacked	
   if(PM&MM) 							Eval-=Paras[37].Val*cl;			// bishop controls stop square  
   PS&=PS-1;															// next pawn	 
  } 
 }
 
 cl=1-2*Gm->color;
 if(((Gm->Count[0]).officers+(Gm->Count[1]).officers==2)&&				// no officers ...
 	((Gm->Count[0]).pawns+(Gm->Count[0]).pawns))						// but pawns
  if(flg&1) 							Eval+=Paras[38].Val*cl;			// vertical opposition bonus depends on stm										
  else if(flg&2) 						Eval+=Paras[39].Val*cl;			// horizontal opposition bonus depends on stm						
  
 Dval=(Oval*gp+Eval*(24-gp))/24;										// tapered evaluation

 if(Gm->color) Val-=Dval; else Val+=Dval;								// evaluation 

 if(Paras[40].Val&&((Val+Paras[40].Val<Alpha)||							// lazy eval
 	(Val-Paras[40].Val>Beta))) goto EVAL_END;
 
 // Mobility and attacks
 
 KM=STEP[1][pk[1-Gm->color]]; MM=STEP[1][pk[Gm->color]];				// dist-1 area around kings
 PM=(Gm->POSITION[Gm->color][0]&(~PP[Gm->color]))|						// stm's officers ...
  	(PP[Gm->color]&(~AP[Gm->color]));									// and undefended pawns
 PN=(Gm->POSITION[1-Gm->color][0]&(~PP[1-Gm->color]))|					// opponent's officers
    	(PP[1-Gm->color]&(~AP[1-Gm->color]));							// and undefended pawns

 NM=(~PP[0])&(~PP[1]); PS=Mv->PINO&NM; PO=Mv->PINS&NM;					// opponent's and stm's pinned officers
 Val+=Paras[41].Val*(Popcount(PS)-Popcount(PO));						// malus for pinned officers  

 NM&=Mv->DCHK; 															// discovered check officers
 PS=NM&Gm->POSITION[Gm->color][0]; PO=NM&Gm->POSITION[1-Gm->color][0];
 Val+=Paras[42].Val*(Popcount(PS)-Popcount(PO));						// bonus for discovered check officers
 
 for(co=1;co<5;co++)													// walk thru moves by piece (no kings or pawns)
 {	
  PPH=MP[co]; PCH=Mv->OATK[co+1]&CM;									// moves by stm and opponent officers 
  Val+=Paras[43].Val*(Popcount(PPH&PN)-Popcount(PCH&PM));				// bonus for officers attacking pieces
  Val+=Paras[44].Val*(Popcount(PPH&KM)-Popcount(PCH&MM));				// bonus for attacks on dist-1 king area
  Val+=Paras[45].Val*(Popcount(PPH&PAW&PP[1-Gm->color])-				// bonus for attacks on weak pawns
  		  Popcount(PCH&PAW&PP[Gm->color])); 

  switch(co)
  {
  	case 1: if(Gm->POSITION[Gm->color][2])								// stm queens exist
	  		{
			 Dval=Popcount(PPH&(~Mv->OATK[0])&(~AP[1-Gm->color]));		// count moves free of attacks
	  		 Val+=Paras[46].Val*Dval; 									// queen mobility
			 if(Dval<3) Val-=Paras[47].Val*(3-Dval);					// malus for trapped queen
	  		}
	  		if(Gm->POSITION[1-Gm->color][2])							// opponent queens exist
			{
			 Dval=Popcount(PCH&(~Mv->ADES)&(~AP[Gm->color]));			// count moves free of attacks
	  		 Val-=Paras[46].Val*Dval; 									// queen mobility
			 if(Dval<3) Val+=Paras[47].Val*(3-Dval);					// bonus for trapped queen
		    }
 			break;
 	case 2: if(Gm->POSITION[Gm->color][3])								// stm rooks exist
 			{
 			 Dval=Popcount(PPH&(~AP[1-Gm->color]));						// count moves free of pawn attacks
 			 Val+=Paras[48].Val*Dval; 									// rook mobility
			 if(Dval<3) Val-=Paras[49].Val*(3-Dval);					// malus for trapped rook
 		    }
	        if(Gm->POSITION[1-Gm->color][3])							// opponent rooks exist
	        {
  			 Dval=Popcount(PCH&(~AP[Gm->color]));						// count moves free of pawn attacks
  			 Val-=Paras[48].Val*Dval; 									// rook mobility
			 if(Dval<3) Val+=Paras[49].Val*(3-Dval); 					// bonus for trapped rook
			}
  			break;
  	case 3: if(Gm->POSITION[Gm->color][4]&WS)							// stm bishop on white squares exists
	  		{
			 Val+=Paras[50].Val*(Dval=Popcount(PPH&(~AP[1-Gm->color])&WS));// count moves free of pawn attacks
  			 if(Dval<3) Val-=Paras[51].Val*(3-Dval);					// malus for trapped bishop
  		    }
			if(Gm->POSITION[Gm->color][4]&BS)							// stm bishop on black squares exists
	  		{
  			 Val+=Paras[50].Val*(Dval=Popcount(PPH&(~AP[1-Gm->color])&BS));// count moves free of pawn attacks
  			 if(Dval<3) Val-=Paras[51].Val*(3-Dval);					// malus for trapped bishop
  		    }
  		    if(Gm->POSITION[1-Gm->color][4]&WS)							// opponent bishop on white squares exists
	  		{
  			 Val-=Paras[50].Val*(Dval=Popcount(PCH&(~AP[Gm->color])&WS));// count moves free of pawn attacks
  			 if(Dval<3) Val+=Paras[51].Val*(3-Dval);					// bonus for trapped bishop
  		    }
  		    if(Gm->POSITION[1-Gm->color][4]&BS)							// opponent bishop on black squares exists
	  		{
			 Val-=Paras[50].Val*(Dval=Popcount(PCH&(~AP[Gm->color])&BS));// count moves free of pawn attacks
  			 if(Dval<3) Val+=Paras[51].Val*(3-Dval);					// bonus for trapped bishop
		    }
  			break;
  	case 4: if(PS=Gm->POSITION[Gm->color][5]) while(PS)					// stm knights exist
	  		{
	  		 sp=find_b[(PS^PS-1)%67]; PS&=PS-1;							// stm knight's position
			 Dval=Popcount(PPH&STEP[2][sp]&(~AP[1-Gm->color]));			// count moves free of pawn attacks
			 Val+=Paras[52].Val*Dval;									// knight mobility 
			 if(Dval<2) Val-=Paras[53].Val*(2-Dval);					// malus for trapped knight	
			}
	  		if(PS=Gm->POSITION[1-Gm->color][5]) while(PS)				// opponent knights exist
  			{
	  		 sp=find_b[(PS^PS-1)%67]; PS&=PS-1;							// opponent knight's position
			 Dval=Popcount(PCH&STEP[2][sp]&(~AP[Gm->color]));			// count moves free of pawn attacks
			 Val-=Paras[52].Val*Dval; 									// knight mobility
			 if(Dval<2) Val+=Paras[53].Val*(2-Dval);					// bonus for trapped knight	
			}
  			break;
  }
 }
 
 Val+=Paras[54].Val*(Popcount((~Mv->OATK[0])&CNTR)-						// malus for uncontrolled center squares
 		 	Popcount((~Mv->ADES)&CNTR));

				 
 // King safety evaluation
 
 Val+=Paras[55].Val*(Popcount(AP[Gm->color]&KM)-						// Pawns attacking king 1 area
 			Popcount(AP[1-Gm->color]&MM));
 
 Oval=Eval=0;															// reinit opening and endgame evaluation
 
 // King evaluation
 	
 // Queen evaluation
 
 for(co=0; co<2; co++)
 {
  PS=Gm->POSITION[co][2]; while(PS)										// all queens
  {
   sp=find_b[(PS^PS-1)%67];												// queen position
	//Oval+=(1-2*co)*(4-Dist[pk[1-co]][sp]);
   PS&=PS-1;
  }
 }
 
 // Rook evaluation
 
 for(co=0; co<2; co++)
 {
  cl=1-2*co; PS=Gm->POSITION[co][3]; while(PS)
  {
   sp=find_b[(PS^PS-1)%67]; CM=PAWN_E[co][0][sp];						// square and spawn of rook
	//Oval+=(1-2*co)*(4-Dist[pk[1-co]][sp]);
   if(!(CM&PP[co]))														// half open file
   {
    Oval+=Paras[56].Val*cl; Eval+=Paras[57].Val*cl;						// opening and endgame bonus
	if(!(CM&PP[1-co]))													// open file
	{
	 Oval+=Paras[58].Val*cl; Eval+=Paras[59].Val*cl;					// opening and endgame bonus
	 if(CM&PS) Oval+=Paras[60].Val*cl;									// double rooks 
    }
   }
   else if(CM&PAP&PP[co]) Eval+=Paras[61].Val*cl;						// rook behind own passer 
   if(PAWN_E[1-co][0][sp]&PAP&PP[1-co]) Eval+=Paras[62].Val*cl;			// rook behind opponent passer 
   
   if((sp>>3)==(1+5*co))												// rook on 7th rank
   {
   	Oval+=Paras[63].Val*cl;												// general opening bonus
   	if((pk[1-co]>>3)==7*co) Oval+=Paras[64].Val*cl;						// bonus for opponent king on 8th rank
   }
   PS&=PS-1;
  }
 }
 
 // Bishop evaluation

 MM=(PP[0]|PP[1]);														// all pawns
 for(co=0; co<2; co++)
 {
  PS=Gm->POSITION[co][4]; cl=1-2*co;
  Eval-=cl*Paras[65].Val*Popcount(PS&WS)*(Popcount(WS&MM)-5);			// malus for pawns on bishop color
  Eval-=cl*Paras[65].Val*Popcount(PS&BS)*(Popcount(BS&MM)-5);			// malus for pawns on bishop color
  while(PS)								
  {
   sp=find_b[(PS^PS-1)%67];												// bishop 
   if((!(PAWN_E[co][6][sp]&PP[1-co]))&&(AP[co]&(A8<<sp)))				// bishop defended and not attackable by pawns
   {
	Eval+=cl*Paras[66].Val;												// bonus for defence by pawn
    if(co) 	{if(sp>31) Oval-=Paras[67].Val;} 							// bonus for outpost
	else 	{if(sp<32) Oval+=Paras[67].Val;}	
   }
   PS&=PS-1;
  } 
 }
 
 // knight evaluation
 
 for(co=0; co<2; co++)
 {
  PS=Gm->POSITION[co][5]; while(PS)										// positions of knights
  {
   sp=find_b[(PS^PS-1)%67];												// square of knight
   if((!(PAWN_E[co][6][sp]&PP[1-co]))&&(AP[co]&(A8<<sp)))				// knight defended and not attackable by pawns
   {
   	Eval+=cl*Paras[68].Val;												// bonus for defence by pawn
    if(co) 	{if(sp>31) Oval-=Paras[69].Val;} 							// bonus for outpost
    else 	{if(sp<32) Oval+=Paras[69].Val;}
   }
   PS&=PS-1;
  }
 }
 
  // pattern evaluation: castles
 
 if(p=(Gm->Moves[Gm->Move_n]).castles)									// calculate castling bonus
 {
  CM=Gm->POSITION[0][4];												// white bishops
  if(((p&0x11)&&(((CSW1&PP[0])==CSW1)||									// kingside castling done or possible
     ((CSW2&PP[0])==CSW2)||((CSW3&(PP[0]|CM))==CSW3)))||				// kingside area is save								
     ((p&0x22)&&(((CLW1&PP[0])==CLW1)||									// queenside castling done or possible
	 ((CLW2&PP[0])==CLW2)||((CLW3&(PP[0]|CM))==CLW3))))					// queenside area is save
	  Oval+=Paras[70].Val;												// castles possible
  if(p&0x30) Oval+=Paras[71].Val;										// castles done	 		
  CM=Gm->POSITION[1][4];												// black pawns and bishops
  if(((p&0x44)&&(((CSB1&PP[1])==CSB1)||									// kingside castling done or possible
  	 ((CSB2&PP[1])==CSB2)||((CSB3&(PP[1]|CM))==CSB3)))||				// kingside area is save								
     ((p&0x88)&&(((CLB1&PP[1])==CLB1)||									// queenside castling done or possible
	 ((CLB2&PP[1])==CLB2)||((CLB3&(PP[1]|CM))==CLB3)))) 				// queenside area is save
	  Oval-=Paras[70].Val;												// castles possible
  if(p&0xC0) Oval-=Paras[71].Val;										// castles done		
 }
 
 // trapped bishop
 
 CM=Gm->POSITION[0][4];													// white bishops
 if(((CM&TWBBH)&&(PP[1]&TWBPH))||((CM&TWBBA)&&(PP[1]&TWBPA))) 			// trapped white bishop
  {Oval-=Paras[72].Val; Eval-=Paras[73].Val;}
  	
 CM=Gm->POSITION[1][4];													// black bishops
 if(((CM&TBBBH)&&(PP[0]&TBBPH))||((CM&TBBBA)&&(PP[0]&TBBPA)))			// trapped black bishop	
 {Oval+=Paras[72].Val; Eval+=Paras[73].Val;}
 
 Dval=(Oval*gp+Eval*(24-gp))/24;										// tapered evaluation
 	
 if(Gm->color) Val-=Dval; else Val+=Dval;								// evaluation 

 EVAL_END:
 *(BitMap*)(ekey)=EHASH^(((BitMap)(Val)&0xFFFF)<<16); *(short*)(ekey)=Val;// store evaluation in hash table
 return Val;
}

short	Qsearch(Game* Gm, short Alpha, short Beta, Byte depth)			// quiescence search
{
 Mvs	Mv;
 Dbyte	Mov,Ply=Gm->Move_n-Gm->Move_r;									// current ply
 short 	Dv,Apriori,Posv,Expect,Bestval,Val;
 Byte	inr,i;

 //return MatEval(Gm);

 if(Ply&1) Dv=5-((short)(Ply)>5?5:(short)(Ply)); else Dv=0;				// additional draw value to accelerate clear draw
 if(Ply>Maxply) Maxply=Ply;												// update maximal ply
 if((Tmax&&((Fbyte)(clock()-StartTime)>Tmax))&&(!Ponder)) Stop=true;	// time exceeds hard break 
 						
 if((Beta<=Alpha)||(Stop)) 					return Alpha;				// no search window or stop recognized
 if(DrawTest(Gm)) 							return Paras[74].Val-Dv;	// 3 or 50 draw
 if(Gm->Move_n) (Gm->Moves[Gm->Move_n-1]).check=TestCheck(Gm,99);		// previous move was check?
 
 if(Options[4].Val)	Apriori=(recog[Gm->Matsig])(Gm,&inr);				// interior node recognition
 else inr=128;															// no INR
 if((inr&128)||(Gm->Moves[Gm->Move_n-1]).check) Apriori=MatEval(Gm);	// evaluation estimate is material value
 else if(!inr)								return Apriori;				// recognizer found exact score
 else if(inr&1)															// upper bound
  {if(Apriori<=Alpha) return Apriori; if(Apriori<Beta)   Beta=Apriori;}	// INR brought fail high or narrowing
 else if(inr&2)															// lower bound
  {if(Apriori>=Beta)  return Apriori; if(Apriori>Alpha) Alpha=Apriori;}	// INR brought fail low or narrowing

 if(Beta<=Alpha)							return Apriori;				// window too small
 if(Gm->phase<12) Posv=Paras[80].Val; else Posv=Paras[79].Val; 			// pruning values for end- and middlegame

 if((Gm->Moves[Gm->Move_n-1]).check)									// in check
 {
  GenMoves(Gm,&Mv);														// generate moves
  if((Mv.cp==64)||(Mv.cp==128)||(Mv.cp==192)) 
  								return Gm->Move_n-Gm->Move_r-MaxScore;	// mate
  else if(!(Mv.cp))							return Paras[74].Val-Dv;	// stalemate
  Bestval=-MaxScore; Mv.flg=0x20;										// init best value and scan all moves
 }
 else
 {
  if(Apriori-Posv>=Beta) 					return Apriori;				// real value will fail high
  if(Gm->POSITION[Gm->color][6]&(P7<<(40*Gm->color))) 					// stm might promote
  		Expect=Paras[95].Val;											// expected promotion value
  else Expect=0;
  if(Apriori+Expect+Paras[5].Val+Posv<=Alpha) return Apriori; 			// even potential queen capture is not enough
  for(i=2;i<7;i++) if(Gm->POSITION[1-Gm->color][i])						// search for maximal possible material gain
  		{Expect+=Paras[3+i].Val; break;}								// most valuable piece
  if(Apriori+Expect+Posv<=Alpha) 			return Apriori; 			// even best possible gain is not enough
  GenMoves(Gm,&Mv);														// generate moves
  if(!(Mv.cp))								return Paras[74].Val-Dv;	// stalemate
  if(!(inr&64))	Apriori=Evaluation(Gm,&Mv,Alpha,Beta);					// if no recognizer get position value, now we know POSV!
  if(Apriori>=Beta)							return Apriori;				// stand pat
  if(Apriori+Expect<=Alpha) 				return Apriori;				// even maximum mat. gain is insufficient
  if(Apriori>Alpha) Alpha=Apriori;										// lower bound
  Bestval=Apriori; 
  if(depth<(Byte)(Paras[81].Val)) Mv.flg=0x40; else Mv.flg=0;			// consider checks
 }
 
 Mv.o=0; Mv.s=100;														// init movepicker
 while(Mov=PickMove(Gm,&Mv))											// next move
 {
  if(!(Mv.flg&0x30))													// no check->delta pruning
  {
   if(Mov&128) Expect=Paras[95].Val; else Expect=0;						// expected promotion value
   if(i=(Gm->Piece[1-Gm->color][(Mov>>8)&63]).type) 
   												Expect=Paras[3+i].Val;	// best expectation
   if((Apriori+Expect+Posv<=Alpha)&&(!TestMCheck(Gm,&Mv,Mov))) continue;// delta pruning if no check
  }
  Move(Gm,Mov); ALLNODES++; 											// make move
  if((level==4)&&(ALLNODES>=MAXNODES)) Stop=true;						// max. nodes in level 4 reached
  Val=-Qsearch(Gm,-Beta,-Alpha,depth+1);								// negamax search, depth increases!
  UnMove(Gm);															// take back move											
  if(Val>Bestval) {Bestval=Val; if(Val>=Beta) return Val;}				// new bestval, fail high cutoff
  if(Val>Alpha) Alpha=Val;					
  if(Stop) 									return Bestval;				// calculation stopped
 }
 											return Bestval;				// return Q-search evaluation
}

void 	GetPV(Game* Gm)													// retrieve principal variation
{																		// from hashtable or principal variation table
 Mvs	Mv;
 BitMap	PM,HM;
 Byte	n=0;
 Dbyte	Mov;
 
 Mv.Bestmove=1;															// initialize move
 while(Mv.Bestmove)														// valid move in hashtable
 {
  GenMoves(Gm,&Mv);														// generate moves
  PM=(Gm->Moves[Gm->Move_n]).HASH; PM^=(PM>>32); PM^=(PM>>16);			// fold hash signature to 16 bit
  Mv.Bestmove=Pv[(Dbyte)(PM)]&0xFFBF; Mv.o=Mv.flg=0; Mv.s=250;			// test move from PV table
  if(!PickMove(Gm,&Mv)) 												// move is not valid. try to get pv move ...
  {
   Mv.Bestmove=0;
   if(HM=GetHash(Gm)) 													// ... from hashtable
   {
    Mv.Bestmove=(Dbyte)(HM>>32)&0xFFBF; Mv.o=Mv.flg=0; Mv.s=250;		// test move from hashtable	
	if(PickMove(Gm,&Mv)) Pv[(Dbyte)(PM)]=Mv.Bestmove; else Mv.Bestmove=0;// move is valid: save move in pv table										  
   }													
  }
  if(Mv.Bestmove&&((!n)||(!DrawTest(Gm))))								// move valid -> update position
  {
   Move(Gm,Mv.Bestmove); n++;	 										// make and count move
   (Gm->Moves[Gm->Move_n-1]).check=TestCheck(Gm,99);					// set check flag
  }
  else (Gm->Moves[Gm->Move_n]).Mov=Mv.Bestmove=0; 						// end of PV
 }
 for(;n;n--) UnMove(Gm);												// take back all moves								
}

void 	PrintPV(Game* Gm, short Val, short Alpha, short Beta)			// print PV
{
 char	Sm[10];
 short	Mm=Gm->Move_r;
 double T2=(double)(clock()-StartTime)/CLOCKS_PER_SEC; 					// overall time passed
 Dbyte	Hf=(HASHFILL*1000)/HEN;											// hash table fill state

 if(Gm->Threadn) return;												// helpers cannot print

 if((Gm->Move_n-Gm->Move_r)&1) {Val=-Val; Alpha=-Alpha; Beta=-Beta;}	// pv in mid of tree
 GetPV(Gm);																// retrieve PV

 if(Options[0].Val) 													// uci mode													
 {
  printf("info depth %d seldepth %d ",Gm->idepth,Maxply);				// depth info
  printf("time %.0lf nodes %llu ",(double)(1000*T2),ALLNODES); 			// time and nodes
  printf("nps %.0lf ",(double)(ALLNODES)/T2);							// nodes per second
  printf("hashfull %d score ",Hf>1000?1000:Hf);							// hashtable fill status
  if(Val>MaxScore-255) 		printf("mate %d ",(MaxScore-Val+1)/2);		// mate in n
  else if(Val<255-MaxScore)	printf("mate -%d ",(Val+MaxScore+1)/2);		// -mate in n 
  else 
  {
   if(Val>9999) Val=9999;												// limit value to 100 Pawns
   else if(Val<-9999) Val=-9999;
   else if((Val>-5)&&(Val<5)) Val=0;									// smoothen draw values
   printf("cp %d ",Val);
  }
  if(Val<=Alpha) 		printf("upperbound ");							// fail low
  else if(Val>=Beta) 	printf("lowerbound ");							// fail high	
  printf("pv ");														// prepare print of pv
 }
 else																	// terminal mode
 {
  printf("Depth: %d/%d ",Gm->idepth,Maxply);							// depth		
  if(Val>MaxScore-255)	printf("value=M%d",(int)((MaxScore-Val+1)/2));	// mate in n
  else if(Val<255-MaxScore)	printf("value=-M%d",(int)((Val+MaxScore)/2));// -mate in n
  else printf("value=%d",Val);											// print ordinary value
  printf(" %llu nodes time=%.0lfs (%.0lf nodes/s)\nPV: ",				// print info
  		ALLNODES,(double)(T2),(double)(ALLNODES)/T2); 	
 }
 while((Gm->Moves[Mm]).Mov) 											// no Nullmoves
  {UncodeMove((Gm->Moves[Mm]).Mov,Sm); printf("%-7s",Sm); Mm++;}		// get move from list
 printf("\n"); fflush(stdout);
}

void 	PrintCurrent(Game* Gm, Dbyte Cm, Dbyte Nn)						// prints current move
{
 char 	Sm[10];

 if(Gm->Threadn) return;												// helpers cannot print
 UncodeMove(Cm,Sm);														// uncode move
 
 if(Options[0].Val)														// UCI mode
  printf("info depth %d currmove %s currmovenumber %d\n",Gm->idepth,Sm,Nn);// print current move
 else printf("current: %-7s\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b",Sm);		// print and move to start of line			
 fflush(stdout);														// flush stdout									
}

bool	TestLine(Game* Gm, Byte d)										// test sequence of moves
{
 Byte 	i=0,j;
 char	Sm[10];
 
 while((i<sizeof(Line)/sizeof(*Line))&&(i<d))
  if(Line[i]!=(Gm->Moves[i]).Mov) return false; else i++;	
 for(j=0;j<i;j++) 
  {UncodeMove(Line[j],Sm); printf("%d: %s\n",j,Sm);}
 
 return true;	  													
}

short	Search(Game* Gm, short Alpha, short Beta, Byte depth, Dbyte* Bestm) // recursive negamax search	
{																	 
 Mvs	Mv,MvB;
 Dbyte	Mov,Bm;
 short 	Val,Hval,Mcval,Bestval,Oalpha,Dv,Ply;
 Byte	r,rm,inr,ext,hd,f,flg,m,n,cm,to,from,cap,type,lmr,hdepth;
 BitMap	HM,PM,NM,KS,RM,NC;
 
 char	Mm[10];

 Oalpha=Alpha; Ply=(short)(Gm->Move_n-Gm->Move_r); Bestval=-MaxScore-10;// initialize values
 hdepth=depth; *Bestm=0; flg=0; inr=128; (Gm->Moves[Gm->Move_n]).Mov=0;	// search flags: 0:multi cut, 1:deep search condition,
 																		// 2:mate threat, 3:nullmove not required, 4:mate value, 
																		// 5:one move only 6:in check, 7: we have an evaluation from TT,IID or IN

 if(Ply&1) Dv=5-(Ply>5?5:Ply); else Dv=0;								// additional draw value to accelerate clear draw
 if(Ply>Maxply) Maxply=Ply;	if(depth>maxdepth) maxdepth=depth;			// update maximal ply
 if((Tmax&&((Fbyte)(clock()-StartTime)>Tmax))&&(!Ponder)) 				// time exceeds hard break 
 	Stop=true;					
 if((Beta<=Alpha)||(Stop)) 					return Alpha;				// no search window or stop recognized
 if(depth&&DrawTest(Gm)) if(Ply) 			return Paras[74].Val-Dv;	// draw according to 3 or 50 moves
 
 if(((Gm->Moves[Gm->Move_n-1]).fifty)&&(Gm->Move_n>2))					// previous move was reversible
  if((Gm->Moves[Gm->Move_n-1]).from==(Gm->Moves[Gm->Move_n-3]).to)		// same piece moves again
  	(Gm->perm)++; else Gm->perm=0;										// sequence broken
 else Gm->perm=0;														// sequence broken
 
 if(depth>6) r=(Byte)(Paras[82].Val); 									// adaptive nullmove reduction
 else r=(Byte)(Paras[82].Val?Paras[82].Val-1:0);	
 PM=(Gm->Moves[Gm->Move_n]).HASH; PM^=(PM>>32); PM^=(PM>>16);			// fold hash signature to 16 bit

 if(HM=GetHash(Gm))														// TT entry found
 {
  hd=(Byte)(HM); f=(Byte)(HM>>8); Val=Hval=(short)(HM>>16);				// get depth, flags, value and move 
  *Bestm=(Dbyte)((HM>>32)&0xFFBF); 							flg|=128;	// we have a TT value and move
  if(Val>MaxScore-255)		{Val-=Ply; 						flg|=16;}	// adjust mate score
  else if(Val<255-MaxScore) {Val+=Ply; 						flg|=16;}
  if(*Bestm)
  {
   if((!(f&1))&&(Val>=Beta)) flg|=1;
   if((!Gm->Move2Make)&&(!Ply)) Gm->Move2Make=*Bestm;					// new root move
   if((hd>1)&&(!Ply)&&(Gm->idepth==1)) 
   					{Gm->idepth=maxdepth=Maxply=hd; return Val;}		// skip unnecessary iterations
  }
  if(depth<=hd)															// hashdepth sufficient
   if(f&1) 		{if(Val<=Alpha) return Val; if(Val<Beta)  Beta=Val;}	// upper bound
   else if(f&2) {if(Val>=Beta)  return Val; if(Val>Alpha) Alpha=Val;}	// lower bound
   else							return Val;								// exact value
  if(Beta<=Alpha)				return Alpha;							// hash cutoff
  if((depth<=hd+1+r)&&(!(f&2))&&(Val<Beta)) 				flg|=8;		// nullmove will not cut off -> nullsearch not necessary
 } 
 
 if(MaxScore-Ply<Beta) 													// mate distance pruning: stm has a mate
  {Beta=MaxScore-Ply;  if(Alpha>=Beta) return Beta;}					// already shorter mate found
 if(Alpha<Ply-MaxScore)													// opponent has a mate
  {Alpha=Ply-MaxScore; if(Beta<=Alpha) return Alpha;}					// opponent has already found a shorter mate

 if(!depth) {Bestval=Qsearch(Gm,Alpha,Beta,0); goto Hash;}				// quiescence search
 
 if((!(*Bestm))&&Paras[84].Val&&(depth>(Byte)(Paras[84].Val)))			// get best move from Internal iterative deepening
 {
  Search(Gm,Alpha,Beta,depth-(Byte)(Paras[84].Val),Bestm);				// search with reduced depth
  *Bestm&=0xFFBF;
 }
 
 if(!(*Bestm)) *Bestm=Pv[(Dbyte)(PM)]&0xFFBF;							// try to get best move from PV hash table

 Mv.Bestmove=*Bestm;
 
 if((!Gm->Move2Make)&&(!Ply)) Gm->Move2Make=*Bestm;						// new root move
 
 GenMoves(Gm,&Mv); NM=Mv.AMVS&(Mv.AMVS-1);								// generate moves
 
 if(!(Mv.cp))								 return Paras[74].Val-Dv;	// stalemate
 if(!NM) 						   							flg|=32;	// one move only?
 if(Gm->Move_n) 
  if(Mv.cp&192) {(Gm->Moves[Gm->Move_n-1]).check=true; 		flg|=64;}	// last move was a check
  else			 (Gm->Moves[Gm->Move_n-1]).check=false;					// last move was no check
 if((Mv.cp==64)||(Mv.cp==128)||(Mv.cp==192)) 							// mate
  							{Bestval=Ply-MaxScore; goto Hash;}
							  							
 if((Ply>2)&&(!(flg&64))&&(Options[4].Val))								// interior node recognizer
 {
  Val=(recog[Gm->Matsig])(Gm,&inr);										// interior node recognition
  if((inr&128)&&((Val>MaxScore-255)||(Val<255-MaxScore))) 	flg|=16;	// mate value
  if(!inr) {*Bestm=0; StoreHash(Gm,Val,0,hdepth,0); return Val;}		// exact score
  //{Bestval=Val; *Bestm=0; goto Hash;}							
  else if(inr&1)														// upper bound
  {
   if(Val<=Alpha) 			{Bestval=Val; *Bestm=0; goto Hash;}			// fail low
   if(Val<Beta)  	Beta=Val; 											// shrink window
  }
  else if(inr&2)														// lower bound
  {
   if(Val>=Beta) 			{Bestval=Val; *Bestm=0; goto Hash;}			// fail high
   if(Val>Alpha) 	Alpha=Val;											// shrink window
   if(Val>Bestval) 	Bestval=Val;										// new best value
  }
  if(Beta<=Alpha)									 return Val;		// window too small
 }

 RM=STEP[1][(Gm->Officer[Gm->color][0]).square];						// region around king 
 KS=RM&Mv.OATK[0]; KS&=KS-1;											// at least 2 attacks on king area
 if(KS&(KS-1)) {KS=RM&(~Mv.OATK[0]); KS&=KS-1; KS&=KS-1;} else KS=1;	// at least 3 attacks on king area and at most 2 save squares
 if((Options[7].Val)&&(!KS)) r--;										// reduce nullmove reduction if king is in danger
 																		// nullmove pruning

 NM&=NM-1; if(!NM) flg|=32; NM&=NM-1;									// at least three legal moves

 if(Paras[82].Val&&Ply&&(Gm->Move_n>2)&&(!(flg&72))&&NM&&((Mv.cp&15)>1)	// nullmove pruning allowed, not at ply0, at least 3 moves...
    &&((Gm->Moves[Gm->Move_n-1]).Mov||(Gm->Moves[Gm->Move_n-2]).Mov))	// ... and 2 pieces, no check, allow silent moves after check																	// double null allowed but not 3 in a row
 {
  Move(Gm,0);															// make nullmove
  if(depth<r+2) 	 Val=-Qsearch(Gm,-Beta,1-Beta,0);					// quiescence search at depth<r+2
  else				 Val=- Search(Gm,-Beta,1-Beta,depth-r-1,&Bm);		// nullsearch with reduction r
  UnMove(Gm);															// take back nullmove
  if((Val<255-MaxScore)) 									flg|=4;		// mate threat
  if((Val<=Alpha-Paras[85].Val)&&(depth==2))							// deep search condition? nullmove fails low and ...
  {
   if(!(flg&128)) {Hval=Evaluation(Gm,&Mv,Alpha,Beta); 	flg|=128;}		// ... evaluation ...
   if(Hval>=Beta)											flg|=2;		// ... fails high
  }
  if((Val>=Beta)&&(Val<=MaxScore-255)) 									// nullmove cutoff
   if(depth>r) depth-=r; else 						return Val;			// verified nullmove							 
 }
																		// multicut pruning
 Mcval=Val=Ply-MaxScore;
 
 if((Paras[83].Val)&&Ply&&(!(flg&80))&&(depth>(Byte)(Paras[83].Val))&&
     (Gm->phase)&&(flg&1))
 {	
  MvB=Mv;  MvB.o=MvB.s=m=cm=0; MvB.flg=0x70; 							// backup move list prepare pickmoves
  while((Mov=PickMove(Gm,&MvB))&&(cm<(Byte)(Paras[86].Val)))			// pick first n moves
  {
   cm++;																// count move
   Move(Gm,Mov); ALLNODES++;											// make move
   if(depth<(Byte)(Paras[83].Val)+2) Val=-Qsearch(Gm,-Beta,1-Beta,0);	// quiescence search
   else	Val=-Search(Gm,-Beta,1-Beta,depth-(Byte)(Paras[83].Val)-1,&Bm);	// test beta cutoff with reduced depth
   UnMove(Gm);															// take back move
   if(Val>Mcval) Mcval=Val;												// best value so far
   if(Val>=Beta) if(++m>=(Byte)(Paras[87].Val)) 	return Mcval;		// multi cutoff
   if(Stop)											return Bestval;		// stop recognized
  }																		// no multicut
 }

 if(!(flg&128)) 			Hval=Evaluation(Gm,&Mv,Alpha,Beta);			// get evaluation if not already done
 Mv.o=Mv.s=n=cm=0; if(depth>4) Mv.flg=0xF0; else Mv.flg=0x70;			// counter moves only for d>4
 
 //if((!Ply)&&(!(Gm->Threadn))) while(Gm->Rootmvs[n]) Gm->Rootmvs[n++]&=0xFFBF;	// clear rootmove processed flags if no helper
 if(!Ply) while(Gm->Rootmvs[n]) Gm->Rootmvs[n++]&=0xFFBF;				// clear rootmove processed flags if no helper

 if((!Ply)&&(Gm->Threadn)==1) Mv.s=1;									// skip hashmove for every second thread

 while(Mov=PickMove(Gm,&Mv))											// loop through moves
 {
  if(!Ply)//&&(!(Gm->Threadn)))
   {rm=0; while(Mov!=Gm->Rootmvs[rm]) rm++; Gm->Rootmvs[rm]|=64;}		// flag root move processed
  if(Stop) 											return Bestval;		// stop detected
  
  to  =(Byte)((Mov>>8)&63); from=(Byte)(Mov&63); 						// origin and destination of move
  type=(Gm->Piece[Gm->color][from]).type;								// moving piece
  cap =(Gm->Piece[1-Gm->color][to]).type;								// captured piece
  if((type==6)&&((from-to)&1)&&(!cap)) cap=6;							// ep capture
  if(TestMCheck(Gm,&Mv,Mov)) Mov|=0x0040; else Mov&=0xFFBF;				// check (some discovered checks are not detected
  ext=0; if(Ply<(short)(2*Gm->idepth))
  {
   n=(Gm->Moves[Gm->Move_n-1]).to;
   if(flg&4)													ext=1;	// mate threat
   else if(Options[5].Val&&(type==6)&&((A8<<to)&PROM7))			ext=1;	// pre-promotion extension
   else if(Options[6].Val&&(Gm->Moves[Gm->Move_n-1]).cap&&(to==n)) ext=1;// recapture on same square
   else if((Mov&64)&&(Ply<=(short)(Gm->idepth)))		 		ext=1;	// check
   else if((Mov&64)&&(Gm->Move_n>2)&&									// check
   		   ((Gm->Moves[Gm->Move_n-2]).check)&&							// repeated check
		   (n!=(Gm->Moves[Gm->Move_n-2]).to))					ext=1;	// not on same square
  }
																		// futility pruning
  if((Alpha>255-MaxScore)&&(Alpha<MaxScore-255)&&cm&&KS&&				// not first move king not in danger
    (Beta>255-MaxScore)&&(Beta<MaxScore-255)&&(depth<5)&&(!ext))		// no mate values not giving check no extension
  {
   Mcval=Alpha-Hval; Val=Hval;											// Alpha - node value
   if(cap)		Mcval-=Paras[cap+3].Val;								// subtract material gain
   if(Mov&128)	Mcval-=Paras[(Mov>>14)+5].Val-Paras[9].Val;				// subtract material gain of promotion
   if(Mcval>Paras[74+depth].Val) 							goto AEL;	// futility pruning
  }
  NC=Gm->NODES;															// backup nodecount
  Move(Gm,Mov);	ALLNODES++;	(Gm->NODES)++;								// make move
  
  if((level==4)&&(ALLNODES>=MAXNODES)) Stop=true;						// nodes exceed limit
  if((!Ply)&&(Gm->idepth>0)) PrintCurrent(Gm,Mov,cm+1);					// print current move at root
  if(!cm)																// pv node / first node
  {
   if(depth+ext<=1) Val=-Qsearch(Gm,-Beta,-Alpha,0);					// quiescence search
   else				Val= -Search(Gm,-Beta,-Alpha,depth+ext-1,&Bm);		// pvs: pv node: normal search
  }
  else
  {
   if((flg&64)||(Mov&64)||(Mv.s<35)||(!Ply)||(!(Paras[88].Val))||		// in check or giving check, PV, tactical, root, no LMR,...
    ((Gm->Count[1-Gm->color]).pawns+(Gm->Count[1-Gm->color]).officers<3)// stm has only one piece/pawn left
	||((Gm->Count[0]).officers<2)||((Gm->Count[1]).officers<2)||		// one side has no pieces left
	(Mv.PINS&(A8<<from))||TestAttk(Gm,type,to,Gm->color)) lmr=0;		// moving a pinned piece or higher piece is attacked-> no lmr
   else 
   {
    lmr=depth/(Byte)(Paras[88].Val); 									// lmr based on depth, move lector state and deep search
	if((Mv.s>50)&&(!(flg&2))) lmr+=(Byte)(Paras[94].Val);
   }	 

   if(depth+ext<=1+lmr) Val=-Qsearch(Gm,-Alpha-1,-Alpha,0);				// null window quiescence search
   else			Val=-Search(Gm,-Alpha-1,-Alpha,depth+ext-1-lmr,&Bm);	// null window search			  										
   if((Val>Alpha)&&(Val<Beta))											// research
    if(depth+ext<=1) Val=-Qsearch(Gm,-Beta,-Alpha,0);					// quiescence research with open window and no lmr
    else			 Val= -Search(Gm,-Beta,-Alpha,depth+ext-1,&Bm);		// normal research with open window and no lmr
  }
  UnMove(Gm);															// take back move
 
AEL:																	// jump here for AEL pruning
  if(!Ply) //&&(!(Gm->Threadn)))											// root but no IID 
  {
   Gm->TREESIZE[rm]=Gm->NODES-NC; // save treesize of root move and count nonloosing moves
   if(Val>50-MaxScore) (Gm->noloose)++;
  }
  cm++;
  if(Stop)											return Bestval;		// stop command recognized
  if(Val>Bestval)														// new best value found
  {
   Bestval=Val; *Bestm=Mov&0xFFBF;										// new best value and best move
   if((!Ply)&&(!(Gm->Threadn)))											// root but no helper
   {
   	Gm->Move2Make=Mov; Pv[(Dbyte)(PM)]=*Bestm; Gm->Lastval=Val;			// backup bestmove so far
   	if(Val>Oalpha) PrintPV(Gm,Bestval,Alpha-1,Beta);					// print new PV
   	Gm->Pmove=(Gm->Moves[Gm->Move_r+1]).Mov;							// ponder move
	if((Val>=MaxScore-(short)(Gm->idepth)-1)&&(!Ponder)&&(level!=1)) 	// shortest mate found, no ponder, no infinite analysis 
							Stop=true;	
   }
   if(Val>=Beta)														// move fails high
   {
   	if((!cap)&&(!(Mov&128)))											// bestmove is quiet, no IID
   	{
   	 Mov&=0xFFBF; 														// clear check flag
	 if(Val>MaxScore-255)												// mate killer
   	  {Gm->Killer[Ply][0][0]=Mov; Gm->Killer[Ply][0][1]=type;}
	 else if((Gm->Killer[Ply][1][0]!=Mov)&&(Gm->Killer[Ply][2][0]!=Mov))// killer move not already stored
	 {
	  Gm->Killer[Ply][2][0]=Gm->Killer[Ply][1][0];						// backup old killer move
	  Gm->Killer[Ply][2][1]=Gm->Killer[Ply][1][1];
	  Gm->Killer[Ply][1][0]=Mov; Gm->Killer[Ply][1][1]=type;			// store new killer move
	 } 
    }
	goto Hash;															// fail high cutoff
   }
  }
  if(Val>Alpha) Alpha=Val;												// adapt alpha
  if(Alpha>=MaxScore-Ply) 							 return Alpha;		// mate distance pruning
  if((!Ply)&&(cm==1)&&(Val<=Oalpha)&&(Gm->idepth>3)) return Val;		// pv move below alpha -> immediate research
 }

Hash:
 
 if((*Bestm)&&(Bestval>Oalpha)&&(Bestval<Beta)&&(!(Gm->Threadn)))		// bestmove exists and pv node, store in pv table	
  Pv[(Dbyte)(PM)]=*Bestm;  												
 Mov=*Bestm; f=0;														// initialize hash move and flags
 if(Bestval<=Oalpha) {f|=1; Mov=0;} else if(Bestval>=Beta) f|=2;		// upper or lower bound
 StoreHash(Gm,Bestval,Mov,hdepth,f);									// store search results in hash table
 if((Stop)||(!(*Bestm)))							return Bestval;		// search was interrupted or no bestmove
 to  =(Byte)(((*Bestm)>>8))&63; from=(Byte)((*Bestm)&63); 				// origin and destination of move
 cap =(Gm->Piece[1-Gm->color][to]).type;								// captured piece
 (Gm->Moves[Gm->Move_n]).from=from; (Gm->Moves[Gm->Move_n]).to=to;		// record bestmove
 (Gm->Moves[Gm->Move_n]).cap=cap;										// move captures?
 (Gm->Moves[Gm->Move_n]).Mov=*Bestm;									// backup compressed move
 if((*Bestm)&128) (Gm->Moves[Gm->Move_n]).prom=(Byte)((*Bestm)>>14)+2;	// move is promotion?
 else 				 (Gm->Moves[Gm->Move_n]).prom=0;
 if((depth>6)&&(!cap)&&(!((*Bestm)&128))&&(Bestval>=Oalpha)&&Gm->Move_n)// update counter table for larger depths only
 {
  to=(Gm->Moves[Gm->Move_n-1]).to&63;									// previous move's destination
  if((Gm->Piece[1-Gm->color][to]).type<6) 								// officer move
   Gm->Counter[Gm->color][(Gm->Piece[1-Gm->color][to]).index][to]=		// store officer counter move
   														*Bestm;
  else Gm->Counter[Gm->color][16][to]=*Bestm;							// store pawn counter move
 }
 return Bestval;
}

void	*SmpSearch(void* Ms)
{
 Game*	Gm;
 
 Gm=(Game*)(Ms); Gm->noloose=0;	Gm->NODES=0;							// set thread parameters
 																		// reset nonloosing move counter
 if(!Stop) 
  Gm->Val=Search(Gm,Gm->Alpha,Gm->Beta,Gm->idepth,&(Gm->Bestmove));		// search with open window
 if(!Stop) 
 {
  PrintPV(Gm,Gm->Val,Gm->Alpha,Gm->Beta); Gm->Lastval=Gm->Val;			// print pv
  	
  if(Gm->Val<=Gm->Alpha)												// fail low
  {
   Gm->noloose=0; 
   Gm->Val=Search(Gm,-MaxScore,Gm->Val+1,Gm->idepth,&(Gm->Bestmove));	// research with half-open lower window
   if(!Stop) PrintPV(Gm,Gm->Val,-MaxScore,MaxScore);					// print search result
  }
  else if(Gm->Val>=Gm->Beta)											// fail high
  {
   Gm->noloose=0; 
   Gm->Val=Search(Gm,Gm->Val-1,MaxScore,Gm->idepth,&(Gm->Bestmove));	// research with half-open upper window
  }
 }
 Gm->Finished=true;														// thread is finished
} 

void	IterateSearch(Game* Gm)											// search for best move
{
 Mvs		Mv;
 BitMap		BM;
 int		i,j,Nm;
 Byte		md;
 Fbyte		Tplan;
 Dbyte		Prevmove,Bm,Bm1;
 Game		Gp[Paras[93].Val];
 
 Gm->Alpha=-MaxScore; Gm->Beta=MaxScore; Gm->Finished=false;            // initialize local values
 md=255; Prevmove=0; Gm->Threadn=0;			
 if((Gm->color&&Binc)||((1-Gm->color)&&Winc)) 							// time increment for every move?
  						Nm=Paras[91].Val; else Nm=Paras[90].Val;		// set number of moves for sudden death
 BM=(Gm->Moves[Gm->Move_n]).HASH; BM^=(BM>>32); BM^=(BM>>16); 			// compress hash signature
 
 StartTime=clock();	Maxply=0; ALLNODES=0; Pv[(Dbyte)(BM)]=0;			// initialize global values
 
 Stop=false; Gm->Move2Make=0; Gm->Lastbest=0; Gm->Pmove=0;				// initialize game parameters
 Gm->Move_r=Gm->Move_n;
 
 if(level!=1) if(Gm->Move2Make=Book(Gm)) 						return;	// get move from opening book
 
 GenMoves(Gm,&Mv); if(!Mv.cp) 									return;	// no moves possible
 Mv.s=200; Mv.o=Mv.flg=i=0;												// prepare move picker
 
 while(Gm->Rootmvs[i]=PickMove(Gm,&Mv)) Gm->TREESIZE[i++]=1;            // init treesizes
 if((i<2)&&(level!=1)&&(!Ponder)) {Gm->Move2Make=Gm->Rootmvs[0]; return;}   // one legal move only
 for(BM=0;BM<2*HEN;BM++) hash_t[16*BM+1]|=0x08;							// set age-flag
 for(i=0;i<256;i++) 
  Gm->Killer[i][0][0]=Gm->Killer[i][1][0]=Gm->Killer[i][2][0]=0;		// clear killer list
 //for(BM=0;BM<   PVN;BM++) 			 Pv[BM]=0; 							// clear pv table
 
 switch(level)															// set time control
 {
  case 0: if(Gm->color) Tmax=Paras[92].Val*Btime/(Movestogo+1)+1; 		// ordinary time control
  		  else 			Tmax=Paras[92].Val*Wtime/(Movestogo+1)+1; break;// hard break at 5-times average
  case 2: if(Gm->color) Tmax=Paras[92].Val*Btime/Nm+1;					// sudden death, estimate x moves to go
  		  else			Tmax=Paras[92].Val*Wtime/Nm+1;			break;	// hard break at 5 times average  
  case 3: md=maxdepth;  Tmax=0;									break;	// depth limit
  case 6: 				Tmax=Movetime;							break;	// hard break is movetime
  default:				Tmax=0;									break;	// infinite: no hard break
 }
 Tmax*=CLOCKS_PER_SEC/1000;												// transform to clock cycles 

 for(i=0;i<Paras[93].Val;i++) 											// initialize threads
 {
  Gp[i]=*Gm;
  Gp[i].Finished=true; Gp[i].idepth=0; Gp[i].Threadn=i+1;
 }
 
 for(Gm->idepth=1;Gm->idepth<=md;(Gm->idepth)++)						// iteration
 {   	
   for(i=0;i<Paras[93].Val;i++) if(Gp[i].Finished)						// parse helper threads
   {
  	if(Gp[i].idepth) pthread_join(Gp[i].Tid,NULL); 						// wait until thread has terminated
	Gp[i].Finished=false;  Gp[i].Alpha=Gm->Alpha; Gp[i].Beta=Gm->Beta;	// initialize new thread
	Gp[i].idepth=Gm->idepth+i/2+1;										// give helper higher depth
	pthread_create(&(Gp[i].Tid), NULL, SmpSearch, (void*)(Gp+i));		// create new helper thread
   } 
 
  SmpSearch((void*)(Gm));												// start main search

  if(Stop) Gm->Val=Gm->Lastval; else Gm->Lastval=Gm->Val;				// backup last value
  if(Gm->idepth>2) 
   {Gm->Alpha=Gm->Val-Paras[89].Val; Gm->Beta=Gm->Val+Paras[89].Val;}	// set aspiration window
  else			   {Gm->Alpha=-MaxScore; Gm->Beta=MaxScore;}			// maximal window
  if(Gm->Val<255-MaxScore) Gm->Alpha=-MaxScore; 						// at most some mate against stm
  if(Gm->Val>MaxScore-255) Gm->Beta=MaxScore;							// at least some mate for stm	
  if(((Gm->idepth>=MaxScore-Gm->Val-1)||(Gm->idepth>=MaxScore+Gm->Val))	// shortest mate found
  	 &&(level!=1)) 								Stop=true;
  if(Gm->color) Tplan=Btime+Binc; else Tplan=Wtime+Winc;				// time left
  Tplan*=CLOCKS_PER_SEC/2000;											// transform to clock cycles
  if(!Ponder) switch(level)												// plan another iteration
  {
   case 0: if((Fbyte)(clock()-StartTime)>(Tplan/(Movestogo+1)))			// ordinary time control
   											Stop=true; 		break;
   case 2: if((Fbyte)(clock()-StartTime)>(Tplan/(Nm+1)))				// sudden death
   											Stop=true; 		break;
   case 3: if(Gm->idepth>=maxdepth)			Stop=true; 		break;		// maximum depth reached
  }
  if(Stop)	{PrintPV(Gm,Gm->Val,-MaxScore,MaxScore); 		return;}	// print pv and return
  if(Gm->Move2Make!=Prevmove) Gm->Lastbest=Prevmove;					// best move has changed
  Prevmove=Gm->Move2Make;												// backup last best move
  if(Stop)													return;		// stop recognized
 } 
}

short 	KvK(Game* Gm, Byte *flags)										// lone kings: checked
{
 short 	dv=Gm->Move_n-Gm->Move_r;
 
 if(dv&1) dv=5-(dv>5?5:dv); else dv=0;									// accelerates draw if inevitable
 
 *flags=0;																// exact score
 return Paras[74].Val-dv;												// draw: insufficient material
}

short 	KPvK(Game* Gm, Byte *flags)										// checked
{
 short 	Val;															// evaluation
 Byte 	wp,mwk,mbk,bk,wk;	
 BitMap AM,BM=Gm->POSITION[0][6];										// white pawn(s)

 Val=(Gm->Count[0]).pawns*Paras[9].Val; AM=BM;							// material value of pawns
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 
 while(BM)
 {
  wp=find_b[(BM^BM-1)%67]; BM&=BM-1;									// pawn position
  if(STEP[7][wp]&AM) {*flags=128; return 0;}							// pawns too close: back off
  Val+=100+Dist_s[wp][bk]-Dist_s[wp][wk]-2*(wp&0xF8)-(wk&0xF8);			// progress
  mwk=wk; mbk=bk;
  if(wp&4)																// pawn on rank e-h
  {
   wp =7-(wp&7)+(wp&0xF8);												// mirror pawn horizontally
   mwk=7-(mwk&7)+(mwk&0xF8); mbk=7-(mbk&7)+(mbk&0xF8);					// mirror kings horizontally			
  }
  if((A8<<mwk)&KPVK[((wp>>3)-1)*4+(wp&3)][Gm->color*64+mbk])			// position is won
  {
   Val+=Paras[5].Val-Paras[9].Val-100;									// value is queen-pawn-promotion bonus
   *flags=0; if(Gm->color) return -Val; else return Val;				// evaluation depends on stm
  }
 }
 if((Gm->Count[0]).pawns==1) {*flags=0; return Paras[74].Val;}			// lone pawn is exactly draw
 if(Gm->color) {*flags=1; return -Val;}									// black has upper bound
 *flags=2; return Val;													// white has lower bound
}

short 	KvKP(Game* Gm, Byte *flags)										// checked
{
 short 	Val;															// evaluation
 Byte 	bp,mwk,mbk,wk,bk;
 BitMap AM,BM=Gm->POSITION[1][6];										// black pawn(s)
 
 Val=(Gm->Count[1]).pawns*Paras[9].Val; AM=BM;							// material value of pawns
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings

 while(BM)
 {
  bp=find_b[(BM^BM-1)%67]; BM&=BM-1;									// pawn position
  if(STEP[7][bp]&AM) {*flags=128; return 0;}							// pawns too close: back off
  Val+=34+Dist_s[bp][wk]-Dist_s[bp][bk]+2*(bp&0xF8)+(bk&0xF8);			// progress
  bp=63-bp; mwk=63-wk; mbk=63-bk;										// mirror all pieces diagonally
  if(bp&4)																// pawn on rank e-h
  {
   bp =7-(bp&7)+(bp&0xF8);												// mirror pawn horizontally
   mwk=7-(mwk&7)+(mwk&0xF8); mbk=7-(mbk&7)+(mbk&0xF8);					// mirror kings horizontally			
  }
  
  if((A8<<mbk)&KPVK[((bp>>3)-1)*4+(bp&3)][(1-Gm->color)*64+mwk])		// position is won
  {
   Val+=Paras[5].Val-Paras[9].Val-100;									// value is queen-pawn-promotion bonus
   *flags=0; if(Gm->color) return Val; else return -Val;				// evaluation depends on stm
  }
 }
 if((Gm->Count[1]).pawns==1) {*flags=0; return Paras[74].Val;}			// lone pawn is exactly draw
 if(Gm->color) {*flags=2; return Val;}									// black has lower bound
 *flags=1; return -Val;													// white has upper bound
}

short 	KNvK(Game* Gm, Byte *flags)										// white has knight(s)
{
 short 	Val;
 Byte 	n,bk,wk;		
 BitMap WN=Gm->POSITION[0][5];											// position of knight(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if((Gm->Count[0]).officers<4) {*flags=0; return Paras[74].Val;}		// less than 3 knights
 Val=10*Dist_c[bk]-5*Dist[wk][bk];										// distance from center and king
 if(Gm->color) *flags=1; else *flags=2;									// white has at least pos value
 if(WN&STEP[1][bk]) *flags=64;											// knight is attacked
 while(WN)																// parse knights
 {
  n=find_b[(WN^WN-1)%67]; WN&=WN-1; 									// next knight
  Val+=38+Paras[8].Val-4*Dist_c[n]-Dist_s[bk][n]; 						// distance of knight to bK and center
 }
 if(Gm->color) return -Val; else return Val;
}

short 	KvKN(Game* Gm, Byte *flags)										// black has knight(s)
{
 short 	Val;
 Byte 	n,bk,wk;				
 BitMap BN=Gm->POSITION[1][5];											// position of knight(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if((Gm->Count[1]).officers<4) {*flags=0; return Paras[74].Val;}		// less than 3 knights
 Val=10*Dist_c[wk]-5*Dist[wk][bk];										// distance from center and king
 if(Gm->color) *flags=2; else *flags=1;									// black has at least pos value
 if(BN&STEP[1][wk]) *flags=64;											// knight is attacked
 while(BN)																// parse knights
 {
  n=find_b[(BN^BN-1)%67]; BN&=BN-1; 									// next knight
  Val+=38+Paras[8].Val-4*Dist_c[n]-Dist_s[wk][n]; 						// distance of knight to wK and center
 }
 if(Gm->color) return Val; else return -Val;
}

short 	KBvK(Game* Gm, Byte *flags)
{
 short 	Val;
 Byte 	bk,wk;															// position of kings
 BitMap BM=Gm->POSITION[0][4];											// position of white bishop(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if(!((BM&WS)&&(BM&BS))) {*flags=0; return Paras[74].Val;}				// lone bishop or all on same color
 Val=50+20*Dist_c[bk]-5*Dist[wk][bk];									// distance of kings, score
 while(BM) {BM&=BM-1; Val+=Paras[7].Val;}								// add material value of bishops				
 if(Gm->color) {*flags=1; Val=-Val;} else *flags=2;						// black has at most pos val maybe lost
 if(STEP[1][bk]&Gm->POSITION[0][0]) *flags=64;							// bishop under attack: no guess
 return Val;
}

short 	KvKB(Game* Gm, Byte *flags)
{
 short 	Val;
 Byte 	wk,bk;															// position of kings
 BitMap BM=Gm->POSITION[1][4];											// position of black bishop(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if(!((BM&WS)&&(BM&BS))) {*flags=0; return Paras[74].Val;}				// lone bishop or all on same color
 Val=50+20*Dist_c[wk]-5*Dist_s[wk][bk];									// distance of kings, score
 while(BM) {BM&=BM-1; Val+=Paras[7].Val;}								// add material value of bishops				
 if(Gm->color) *flags=2; else {*flags=1; Val=-Val;}						// black has at least pos val maybe won
 if(STEP[1][wk]&Gm->POSITION[1][0]) *flags=64;							// bishop under attack: no guess
 return Val;															// white has at most pos valmaybe lost
}

short 	KRvK(Game* Gm, Byte *flags)										// also KR(B/N)vK
{
 short 	Val;
 Byte 	bk,wk;															// position of kings
 BitMap BM=Gm->POSITION[0][3];											// position of white rook(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 Val=120+20*Dist_c[bk]-5*Dist_s[bk][wk];								// distance of kings, score
 while(BM) {Val+=20+Paras[6].Val; BM&=BM-1;}							// add material value of rooks
 BM=Gm->POSITION[0][4]; while(BM) {Val+=20+Paras[7].Val; BM&=BM-1;}		// add material value of bishops
 BM=Gm->POSITION[0][5]; while(BM) {Val+=20+Paras[8].Val; BM&=BM-1;}		// add material value of knights
 if(Gm->color) {*flags=1; Val=-Val;} else *flags=2;						// white has at least pos val maybe won
 if(STEP[1][bk]&Gm->POSITION[0][0]) *flags=64;							// pieces under attack: no guess
 return Val;															// black has at most pos val maybe lost
}

short 	KvKR(Game* Gm, Byte *flags)										// also KvKR(B/N)
{
 short 	Val;
 Byte 	wk,bk;															// position of kings
 BitMap BM=Gm->POSITION[1][3];											// position of black rook(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 Val=120+20*Dist_c[wk]-5*Dist_s[bk][wk];								// distance of kings, score
 while(BM) {Val+=20+Paras[6].Val; BM&=BM-1;}							// add material value of rooks
 BM=Gm->POSITION[1][4]; while(BM) {Val+=20+Paras[7].Val; BM&=BM-1;}		// add material value of bishops
 BM=Gm->POSITION[1][5]; while(BM) {Val+=20+Paras[8].Val; BM&=BM-1;}		// add material value of knights
 if(Gm->color) *flags=2; else {*flags=1; Val=-Val;}						// black has at least pos val maybe won
 if(STEP[1][wk]&Gm->POSITION[1][0]) *flags=64;							// pieces under attack: no guess
 return Val;															// white has at most pos val maybe lost
}

short 	KQvK(Game* Gm, Byte *flags)										// also KQ(R/B/N)vK
{
 short 	Val;
 Byte	bk,wk;															// position of kings
 BitMap BM=Gm->POSITION[0][2];											// position of white queen(s)
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 Val=200+15*Dist_c[bk]-10*Dist_s[bk][wk];								// distance of kings, score
 while(BM) {Val+=20+Paras[5].Val; BM&=BM-1;}							// add material value of queens
 BM=Gm->POSITION[0][3]; while(BM) {Val+=20+Paras[6].Val; BM&=BM-1;}		// add material value of rooks
 BM=Gm->POSITION[0][4]; while(BM) {Val+=20+Paras[7].Val; BM&=BM-1;}		// add material value of bishops
 BM=Gm->POSITION[0][5]; while(BM) {Val+=20+Paras[8].Val; BM&=BM-1;}		// add material value of knights
 if(Gm->color) {*flags=1; Val=-Val;} else *flags=2;						// black has at least pos value
 if(STEP[1][bk]&Gm->POSITION[0][0]) *flags=64;							// pieces under attack: no guess
 return Val;															// black has at most pos val maybe lost
}

short 	KvKQ(Game* Gm, Byte *flags)										// also KvKQ(R/B/N)
{
 short 	Val;
 Byte 	wk,bk;															// position of kings
 BitMap BM=Gm->POSITION[1][2];											// position of black queen(s)

 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 Val=200+15*Dist_c[wk]-10*Dist_s[bk][wk];								// distance of kings, score
 while(BM) {Val+=20+Paras[5].Val; BM&=BM-1;}							// add material value of queens
 BM=Gm->POSITION[1][3]; while(BM) {Val+=20+Paras[6].Val; BM&=BM-1;}		// add material value of rooks
 BM=Gm->POSITION[1][4]; while(BM) {Val+=20+Paras[7].Val; BM&=BM-1;}		// add material value of bishops
 BM=Gm->POSITION[1][5]; while(BM) {Val+=20+Paras[8].Val; BM&=BM-1;}		// add material value of knights
 if(Gm->color) *flags=2; else {*flags=1; Val=-Val;}						// black has at least pos value
 if(STEP[1][wk]&Gm->POSITION[1][0]) *flags=64;							// pieces under attack: no guess
 return Val;															// white has at most pos val maybe lost
}

short 	KBNvK(Game* Gm, Byte *flags)									// white has bishop(s) and knight(s)
{
 short 	Val;															// evaluation
 Byte 	n,bk,wk;														// position of kings
 BitMap WB=Gm->POSITION[0][4],WN=Gm->POSITION[0][5];					// position of bishops and knights
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if(WB&WS) Val=Dist_b[0][bk]; else Val=Dist_b[1][bk];					// distance of bK to bishop's corner
 Val=250-15*Val-4*Dist_s[wk][bk];										// distance of kings
 if(Gm->color) *flags=1; else *flags=2;									// white has at least pos value
 if((WN|WB)&STEP[1][bk]) *flags=64;										// bishop or knight attacked
 while(WN)																// parse knights
 {
  n=find_b[(WN^WN-1)%67]; WN&=WN-1;										// next knight
  Val+=34+Paras[8].Val-Dist_s[bk][n]; 									// material and distance of knight to bK
 }
 while(WB) {Val+=20+Paras[7].Val; WB&=WB-1;}							// add bishop's material value	
 if(Gm->color) return -Val; else return Val;							// black has at most pos val maybe lost
}

short 	KvKBN(Game* Gm, Byte *flags)									// black has bishop(s) and knight(s)
{
 short 	Val;															// evaluation
 Byte 	n,bk,wk;														// position of kings
 BitMap BB=Gm->POSITION[1][4],BN=Gm->POSITION[1][5];					// position of bishops and knights
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if(BB&WS) Val=Dist_b[0][wk]; else Val=Dist_b[1][wk];					// distance of wK to bishop's corner
 Val=250-15*Val-4*Dist_s[wk][bk];										// distance of kings
 if(Gm->color) *flags=2; else *flags=1;									// black has at least pos value
 if((BN|BB)&STEP[1][wk]) *flags=64;										// bishop or knight attacked
 while(BN)																// parse knights
 {
  n=find_b[(BN^BN-1)%67]; BN&=BN-1;										// next knight
  Val+=34+Paras[8].Val-Dist_s[wk][n]; 									// material and distance of knight to wK
 }
 while(BB) {Val+=20+Paras[7].Val; BB&=BB-1;}							// add bishop's material value	
 if(Gm->color) return Val; else return -Val;							// white has at most pos val maybe lost
}

short 	KNPvK(Game* Gm, Byte *flags)
{
 BitMap WP=Gm->POSITION[0][6];											// position of white pawns
 
 if(((Gm->Count[0]).officers==2)&&((Gm->Count[0]).pawns==1)&&(WP&A7H7)&&
    ((WP>>8)&Gm->POSITION[1][1])) {*flags=0; return Paras[74].Val;}  	// exact draw
 
 *flags=128; return 0;													// no guess
}

short 	KvKNP(Game* Gm, Byte *flags)
{
 BitMap BP=Gm->POSITION[1][6];											// position of black pawns
 
 if(((Gm->Count[1]).officers==2)&&((Gm->Count[1]).pawns==1)&&(BP&A2H2)&&
    ((BP<<8)&Gm->POSITION[0][1])) {*flags=0; return Paras[74].Val;}  	// exact draw
 
 *flags=128; return 0;													// no guess
}

short 	KBPvK(Game* Gm, Byte *flags)
{
 Byte 	wk,bk,wps,p,n;													
 BitMap WB,WP,BKB;			
 short 	Val;
 
 *flags=128;															// default is no guess
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 WB=Gm->POSITION[0][4]; WP=Gm->POSITION[0][6]; BKB=Gm->POSITION[1][1];	// position of white bishops and pawns
 wps=find_b[(WP^WP-1)%67]; n=7-(wps>>3);								// square of most advanced pawn
 
 if(((Gm->Count[0]).officers>2)||(((~A)&WP)&&((~H)&WP))) return 0;		// more than one bishop or pawns not on a/h files
 *flags=64;																// default is unclear
 Val=3*n*n+(Gm->Count[0]).pawns*Paras[9].Val;							// value of pawn(s)
 Val+=2*Dist[bk][wps]-Dist[wk][wps];									// distance to pawn
 if(Gm->color) p=0; else p=8;											// stm correction for Berger
 if((!(BKB&PAWN_E[0][2][wps-p]))&&
    (!(Gm->POSITION[0][0]&PAWN_E[0][0][wps]))) Val+=600;				// king not in Berger's square
 if(BKB&PAWN_E[0][0][wps]) Val-=100;									// king on pawn's spawn
 if(((wps&1)&&(WB&BS))||((!(wps&1))&&(WB&WS))) Val+=200;				// bishop controls promotion square
 else 
 {
  p=Dist[bk][wps&7]; Val+=10*p;											// distance to promotion square
  if(p<1) {*flags=0; return Paras[74].Val;}								// king controls promotion square exact draw
 }
 if(Gm->color) return -Val; else return Val;
}

short 	KvKBP(Game* Gm, Byte *flags)
{
 Byte 	wk,bk,bps,p,n;													
 BitMap BB,BP,WKB;			
 short 	Val;
 
 *flags=128;															// default is no guess
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 BB=Gm->POSITION[1][4]; BP=Gm->POSITION[1][6]; WKB=Gm->POSITION[0][1];	// position of black bishops and pawns
 
 if(((Gm->Count[1]).officers>2)||(((~A)&BP)&&((~H)&BP))) return 0;		// more than one bishop or pawns not on a/h files
 *flags=64;																// default is unclear
 while(BP) {bps=find_b[(BP^BP-1)%67]; BP&=BP-1;}						// most advanced pawn
 n=bps>>3;																// rank of that pawn															
 Val=3*n*n+(Gm->Count[1]).pawns*Paras[9].Val;							// value of pawn(s)
 Val+=2*Dist[wk][bps]-Dist[bk][bps];									// distance to pawn
 
 if(Gm->color) p=8; else p=0;											// stm correction for Berger
 if((!(WKB&PAWN_E[1][2][bps+p]))&&
    (!(Gm->POSITION[1][0]&PAWN_E[1][0][bps]))) Val+=600;				// king not in Berger's square
 if(WKB&PAWN_E[1][0][bps]) Val-=100;									// king on pawn's spawn
 if(((bps&1)&&(BB&WS))||((!(bps&1))&&(BB&BS))) Val+=200;				// bishop controls promotion square
 else 
 {	
  p=Dist[wk][56+(bps&7)]; Val+=10*p; 									// distance to promotion square
  if(p<1) {*flags=0; return Paras[74].Val;}								// king controls promotion square exact draw
 }
 if(Gm->color) return Val; else return -Val;
}

short 	KPvKB(Game* Gm,Byte *flags)
{
 short 	Val;															// evaluation
 BitMap BM=Gm->POSITION[1][4],PM=Gm->POSITION[0][6];					// position of black bishops and white pawns
 Byte 	wk,bk,wps=find_b[(PM^PM-1)%67],bbs=find_b[(BM^BM-1)%67];		// pawn and bishop square
 Byte 	n=7-(wps>>3);													// pawn's rank
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if(((BM&WS)&&(BM&BS))||((Gm->Count[0]).pawns>1)||						// more than one bishop or pawn or
 	(Gm->POSITION[0][1]&(PM>>8)&COR))	{*flags=128; return 0;}			// wK trapped: back off 
 if(!(Gm->POSITION[1][1]&COR))											// bK not in corner
 {
  Val=KPvK(Gm,flags); 													// check position without bishop
  if((Val==Paras[74].Val)&&(*flags==0)) return Val; 					// position draw even without bishop
 }											
 Val=2*Dist_s[bk][wps]-Dist_s[wk][wps]-Paras[7].Val;					// material value of bishop
 Val+=3*n*n+Paras[9].Val;												// value of pawn
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(Gm->color) {*flags=1; Val=-Val;} else *flags=2;
 if(BM&(STEP[1][wk]|STEP[3][wps]))			*flags=64;					// bishop is attacked
 else if((PAWN_E[0][0][wps]&STEP[11][bbs])&&							// bishop controls pawn's spawn
  			(!(STEP[11][bbs]&Gm->POSITION[1][1]))) 						// and is not blocked by king
  						{*flags=0; return Paras[74].Val;}				// exact draw
 return Val;
}

short 	KBvKP(Game* Gm,Byte *flags)
{
 short 	Val;															// evaluation
 BitMap BM=Gm->POSITION[0][4],PM=Gm->POSITION[1][6];					// position of black bishops and white pawns
 Byte 	wk,bk,bps=find_b[(PM^PM-1)%67],wbs=find_b[(BM^BM-1)%67];		// pawn and bishop square
 Byte 	n=bps>>3;														// pawn's rank
 
 bk=(Gm->Officer[1][0]).square;	wk=(Gm->Officer[0][0]).square;			// position of kings
 if(((BM&WS)&&(BM&BS))||((Gm->Count[1]).pawns>1)||						// more than one bishop or pawn or
 	(Gm->POSITION[1][1]&(PM<<8)&COR))	{*flags=128; return 0;}			// wK trapped: back off 
 if(!(Gm->POSITION[0][1]&COR))											// bK not in corner
 {
  Val=KvKP(Gm,flags); 													// check position without bishop
  if((Val==Paras[74].Val)&&(*flags==0)) return Val; 					// position draw even without bishop
 }											
 Val=2*Dist_s[wk][bps]-Dist_s[bk][bps]-Paras[7].Val;					// material value of bishop
 Val+=3*n*n+Paras[9].Val;												// value of pawn
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(Gm->color) *flags=2; else {*flags=1; Val=-Val;}
 if(BM&(STEP[1][bk]|STEP[4][bps]))			*flags=64;					// bishop is attacked
 else if((PAWN_E[1][0][bps]&STEP[11][wbs])&&							// bishop controls pawn's spawn
  			(!(STEP[11][wbs]&Gm->POSITION[0][1]))) 						// and is not blocked by king
  						{*flags=0; return Paras[74].Val;}				// exact draw
 return Val;
}

short 	KPvKN(Game* Gm, Byte *flags)
{
 short 	Val;															// evaluation
 BitMap BN=Gm->POSITION[1][5],WP=Gm->POSITION[0][6];					// position of knights and pawns
 Byte 	bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square,n;	// square of kings
 Byte   wps=find_b[(WP^WP-1)%67],d=7-(wps>>3);							// square and rank of pawn
 
 *flags=128;															// default is back off
 if((WP&(WP-1))||((Gm->Count[1]).officers>3)) 			return 0;		// more than one pawn or two knights
 if(Gm->POSITION[0][1]&(WP>>8)&COR)						return 0;		// king trapped
 Val=4*d*d;																// progress of pawn 
 if((Gm->Count[1]).officers>2) 											// more than one knight
 {
  Val-=10*Dist_c[wk]-5*Dist[wk][bk]; *flags=64;
  if((Gm->Piece[1][wps-8]).type==5) Val-=50;							// drive king to corner and block pawn
 }
 else 
 {
  Val+=2*Dist[bk][wps]-Dist[wk][wps];
  if(!(BN&(STEP[1][wk]|STEP[3][wps])))									// knight not attacked
   if(PAWN_E[0][0][wps]&Gm->POSITION[1][1]) 							// king on pawn's spawn
    							{*flags=0; return Paras[74].Val;}		// exact draw
  Val+=5*Dist[wps][find_b[(BN^BN-1)%67]];								// distance of knight
  Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least draw
  if(Gm->color) *flags=1; else *flags=2;								// white has at least pos value maybe won
 }
 if(BN&(STEP[1][wk]|STEP[3][wps]))						*flags=64;		// knight attacked 
 if(!(PAWN_E[0][2][wps]&Gm->POSITION[1][0]))			*flags=64;		// no piece in Berger's square 
 if(Gm->color) 						return -Val; else return Val;
}

short 	KNvKP(Game* Gm, Byte *flags)
{
 short 	Val;															// evaluation
 BitMap WN=Gm->POSITION[0][5],BP=Gm->POSITION[1][6];					// position of knights and pawns
 Byte 	bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square,n;	// square of kings
 Byte   bps=find_b[(BP^BP-1)%67],d=(bps>>3);							// square and rank of pawn
 
 *flags=128;															// default is back off
 if((BP&(BP-1))||((Gm->Count[0]).officers>3)) 			return 0;		// more than one pawn or two knights
 if(Gm->POSITION[1][1]&(BP<<8)&COR)						return 0;		// king trapped
 Val=4*d*d;																// progress of pawn 
 if((Gm->Count[0]).officers>2) 											// more than one knight
 {
  Val-=10*Dist_c[bk]-5*Dist[wk][bk]; *flags=64;
  if((Gm->Piece[0][bps+8]).type==5) Val-=50;							// drive king to corner and block pawn
 }
 else 
 {
  Val+=2*Dist[wk][bps]-Dist[bk][bps];
  if(!(WN&(STEP[1][bk]|STEP[4][bps])))									// knight not attacked
   if(PAWN_E[1][0][bps]&Gm->POSITION[0][1]) 							// king on pawn's spawn
    							{*flags=0; return Paras[74].Val;}		// exact draw
  Val+=5*Dist[bps][find_b[(WN^WN-1)%67]];								// distance of knight
  Val=Val<Paras[74].Val?Paras[74].Val:Val;								// black has at least draw
  if(Gm->color) *flags=2; else *flags=1;								// black has at least pos value maybe won
 }
 if(WN&(STEP[1][bk]|STEP[4][bps]))						*flags=64;		// knight attacked 
 if(!(PAWN_E[1][2][bps]&Gm->POSITION[0][0]))			*flags=64;		// no piece in Berger's square 
 if(Gm->color) 						return Val; else return -Val;
}

short 	KQvKP(Game* Gm, Byte *flags)
{
 short 	Val;															// evaluation
 BitMap BM=Gm->POSITION[0][2],PM=Gm->POSITION[1][6];					// position of white queen(s) and black pawn(s)
 BitMap KM=Gm->POSITION[1][1];	
 Byte 	wk=(Gm->Officer[0][0]).square,n;
 
 *flags=128; n=find_b[(PM^PM-1)%67];									// default is back off
 if((BM&(BM-1))||(PM&(PM-1))||(!(PM&PBQ))) 		 	return 0;			// more than 1 queen or pawn or not on crit. squares
 if(Dist[wk][n]<4) 							 		return 0;			// wK too close to pawn
 if(((n==48)&&(KM&PBKA))||((n==55)&&(KM&PBKH))||						// King on save squares
    ((n==50)&&(KM&PBKA))||((n==53)&&(KM&PBKH))) 
	{if(Gm->color) *flags=2; else *flags=1; Val=70-10*Dist[wk][n];}
 else
 {
  Val=Paras[5].Val-Paras[9].Val-20*Dist[wk][n];							// progress of wk
  Val-=4*(n>>3)*(n>>3); *flags=64;										// progress of black pawn	
 }
 if(Gm->color) return -Val; else return Val;
}

short	KPvKQ(Game* Gm, Byte *flags)
{
 short 	Val;															// evaluation
 BitMap BM=Gm->POSITION[1][2],PM=Gm->POSITION[0][6];					// position of white queen(s) and black pawn(s)
 BitMap	KM=Gm->POSITION[0][1];	
 Byte 	bk=(Gm->Officer[1][0]).square,n;
 
 *flags=128; n=find_b[(PM^PM-1)%67];									// default is back off
 if((BM&(BM-1))||(PM&(PM-1))||(!(PM&PWQ))) 			return 0;			// more than 1 queen or pawn or not on crit. squares
 if(Dist[bk][n]<4)  								return 0;			// bK too close to pawn or pawn not defended by wK
 if(((n==8)&&(KM&PWKA))||((n==15)&&(KM&PWKH))||				
    ((n==10)&&(KM&PWKC))||((n==13)&&(KM&PWKF))) 						// PWKC PWKF
	 {*flags=0; Val=Paras[74].Val;}
 else
 {
  Val=Paras[5].Val-Paras[9].Val-20*Dist[bk][n];							// progress of bK
  n=7-(n>>3); Val-=4*n*n; *flags=64;									// progress of wP
 }
 if(Gm->color) return Val; else return -Val;										
}

short 	KPvKP(Game* Gm,Byte *flags)										// checked
{
 short Val,V;															// evaluation
 Byte 	wp,bp,mwk;
 Byte	bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap WPB=Gm->POSITION[0][6],BPB=Gm->POSITION[1][6];					// bitmap of pawns
 
 *flags=128; if((Gm->Count[0]).pawns+(Gm->Count[1]).pawns>2) return 0;	// more than 2 pawns
 wp=find_b[(WPB^WPB-1)%67]; bp=find_b[(BPB^BPB-1)%67];					// squares of pawns
 if(!(PAWN_E[0][0][wp]&BPB)) 					return 0;				// pawns not on same file
 if((STEP[1][wk]&BPB)||(STEP[1][bk]&WPB))		return 0;				// pawns attacked
 Val=Dist[bk][wp]+Dist[bk][bp]-Dist[wk][wp]-Dist[wk][bp];				// distance of kings
 Val+=10*((7-(wp>>3))-(bp>>3));	V=Val;									// progress
 if((Val>18)||(Val<-18)) Val*=20;										// king far away
 if(!(WPB&(BPB<<8))) 													// pawns not blocked
  {*flags=64; if(Gm->color) return -Val; else return Val;}				// return distance value
 if(wp&4)																// pawns on rank e-h
 {
  wp=7-(wp&7)+(wp&0xF8); bp=7-(bp&7)+(bp&0xF8);							// mirror pawns horizontally
  wk=7-(wk&7)+(wk&0xF8); bk=7-(bk&7)+(bk&0xF8);							// mirror kings horizontally			
 }
 *flags=0;																// exact value
 if((A8<<wk)&KPVKP[((wp>>3)-2)*4+(wp&3)][Gm->color*64+bk])				// position is won for stm
 { 
  V+=Paras[5].Val-2*Paras[9].Val;										// value is queen-pawn-promotion bonus
  if(Gm->color) return -V; else return V;								// evaluation depends on stm
 }
 wp=8*(7-(bp>>3))+(bp&7); mwk=wk;                      					// swap pawns and mirror vertically
 wk=8*(7-(bk>>3))+(bk&7); bk=8*(7-(mwk>>3))+(mwk&7);					// swap kings and mirror vertically
 
 if((A8<<wk)&KPVKP[((wp>>3)-2)*4+(wp&3)][(1-Gm->color)*64+bk])			// position is won for opponent
 { 
  V-=Paras[5].Val-2*Paras[9].Val;										// value is queen-pawn-promotion bonus
  if(Gm->color) return -V; else return V;								// evaluation depends on stm
 }
 return Paras[74].Val;													// position is draw        
}

short 	KRBvKR(Game* Gm, Byte *flags)
{ 
 short	Val;
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3],WB=Gm->POSITION[0][4];// position of rooks and bishop
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte   wrs=find_b[(WR^WR-1)%67],brs=find_b[(BR^BR-1)%67];				// squares of rooks
 Byte	wbs=find_b[(WB^WB-1)%67];										// square of bishop
 
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>5) 
 												{*flags=128; return 0;}	// more than two rooks and one bishop
 Val=0+10*Dist_c[bk]-5*Dist[wk][bk];
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(Gm->color) {*flags=1; Val=-Val;} else 		*flags=2;				// white has at least pos value	
 if(BR&(STEP[1][wk]|STEP[11][wbs]))				*flags=64;				// black rook attacked
 if((WR|WB)&(STEP[1][bk]|STEP[10][brs]))		*flags=64;				// white rook or bishop attacked
 return Val;
}

short 	KRvKRB(Game* Gm, Byte *flags)
{ 
 short	Val;
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3],BB=Gm->POSITION[1][4];// position of rooks and bishop
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte   wrs=find_b[(WR^WR-1)%67],brs=find_b[(BR^BR-1)%67];				// squares of rooks
 Byte	bbs=find_b[(BB^BB-1)%67];										// square of bishop
 
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>5) 
 												{*flags=128; return 0;}	// more than two rooks and one bishop
 Val=0+10*Dist_c[wk]-5*Dist[wk][bk];
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// black has at least a draw
 if(Gm->color) *flags=2;  else 					{*flags=1; Val=-Val;}	// black has at least pos value	
 if(WR&(STEP[1][bk]|STEP[11][bbs]))				*flags=64;				// white rook attacked
 if((BR|BB)&(STEP[1][wk]|STEP[10][wrs]))		*flags=64;				// black rook or bishop attacked
 return Val;
}

short 	KRNvKR(Game* Gm, Byte *flags)
{
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3];					// position of rooks and knight
 BitMap WN=Gm->POSITION[0][5];	
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte   wrs=find_b[(WR^WR-1)%67],brs=find_b[(BR^BR-1)%67];				// squares of rooks
 Byte	wns=find_b[(WN^WN-1)%67];										// square of knight
 short	Val;
 
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>5) 
 												{*flags=128; return 0;}	// more than two rooks and one bishop
 Val=0+10*Dist_c[bk]-5*Dist[wk][bk];
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(Gm->color) {*flags=1; Val=-Val;} else *flags=2;						// white has at least pos value	
 if(BR&(STEP[1][wk]|STEP[2][wns]))				*flags=64;				// black rook attacked
 if((WR|WN)&(STEP[1][bk]|STEP[10][brs]))		*flags=64;				// white rook or knight attacked
 return Val;
}

short 	KRvKRN(Game* Gm, Byte *flags)
{
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3];					// position of rooks and knight
 BitMap BN=Gm->POSITION[1][5];	
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte   wrs=find_b[(WR^WR-1)%67],brs=find_b[(BR^BR-1)%67];				// squares of rooks
 Byte	bns=find_b[(BN^BN-1)%67];										// square of knight
 short	Val;
 
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>5) 
 												{*flags=128; return 0;}	// more than two rooks and one bishop
 Val=0+10*Dist_c[wk]-5*Dist[wk][bk];
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// black has at least a draw
 if(Gm->color) *flags=2; 				else {*flags=1; Val=-Val;}		// black has at least pos value	
 if(WR&(STEP[1][bk]|STEP[2][bns]))				*flags=64;				// white rook attacked
 if((BR|BN)&(STEP[1][wk]|STEP[10][wrs]))		*flags=64;				// black rook or knight attacked
 return Val;
}

short 	KBvKB(Game* Gm, Byte *flags)
{
 *flags=128; 															// default is back off
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4)		return 0;	// one side has more than one bishop
 *flags=0; return Paras[74].Val;										// position is draw
}

short 	KNvKN(Game* Gm, Byte *flags)
{
 *flags=128; 															// default is back off
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4)		return 0;	// one side has more than one 
 *flags=0; return Paras[74].Val;										// position is draw
}

short 	KRvKR(Game* Gm, Byte *flags)
{
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3];					// position of rooks
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte   wrs=find_b[(WR^WR-1)%67],brs=find_b[(BR^BR-1)%67];				// squares of rooks

 *flags=64;																// default is back off
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4) 
 											{*flags=128; return 0;}		// more than one piece per side
 if((WR&(STEP[1][bk]|STEP[10][brs]))||(BR&(STEP[1][wk]))||				// rook attacked
	(BOR&(Gm->POSITION[0][1]|Gm->POSITION[1][1]))) *flags=64;			// king at border
 return Paras[74].Val; 
}

short 	KQvKQ(Game* Gm, Byte *flags)
{
 BitMap WQ=Gm->POSITION[0][2],BQ=Gm->POSITION[1][2];					// position of queens
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte   wqs=find_b[(WQ^WQ-1)%67],bqs=find_b[(BQ^BQ-1)%67];				// position of queens

 *flags=64;																// default is back off
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4) 
 											{*flags=128; return 0;}		// more than one piece per side
 if((WQ&(STEP[1][bk]|STEP[10][bqs]|STEP[11][bqs]))||					// white queen attacked
    (BQ&(STEP[1][wk]))||												// black queen attacked
	(BOR&(Gm->POSITION[0][1]|Gm->POSITION[1][1]))) *flags=64;			// king at border
 return Paras[74].Val;   												// position is draw
}

short 	KBPvKB(Game* Gm, Byte *flags)
{
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap WP=Gm->POSITION[0][6],BB=Gm->POSITION[1][4],WB=Gm->POSITION[0][4];// position of pawn and bishops
 Byte 	wps=find_b[(WP^WP-1)%67],wbs=find_b[(WB^WB-1)%67];				// square of pawn and bishop
 BitMap PS=PAWN_E[0][0][wps];											// pawn's spawn
 Byte 	bbs=find_b[(BB^BB-1)%67],pr=wps&7;								// promotion square
 short 	Val;									
 
 *flags=128;															// default is back off
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers+					// more than one piece or pawn per side
 								(Gm->Count[0]).pawns>5) return 0;	
 if(((WB&WS)&&(BB&WS))||((WB&BS)&&(BB&BS))) 	 		return 0;		// bishops on same color
 Val=2*Square[0][5][1][wps]+20*Dist[bk][pr]-5*Dist[wk][pr];				// pawn's progress and distance of kings
 *flags=0;																// exact score
 if(BB&(STEP[1][wk]|STEP[11][wbs]|STEP[3][wps])) 	*flags=64; 			// bishop attacked
 if((WB|WP)&(STEP[1][bk]|STEP[11][bbs]))			*flags=64;			// bishop or pawn attacked
 if(WP&(P6|P7))										*flags=64;  		// pawn too far advanced	
 if(STEP[11][bbs]&PS) 													// bishop attacks pawn's spawn
 {
  if(!((BB&WB&BS)||(BB&WB&WS)))					return Paras[74].Val;	// bishops on different colours
  Val-=15;																// malus for bishop controls spawn 			
 }
 if(Gm->POSITION[1][1]&PS)												// king is on pawn's spawn
 {
  if(Gm->POSITION[1][1]&WS) {if(WB&BS) 			return Paras[74].Val;}	// king on spawn and not on bishop color
  else if(WB&WS) 								return Paras[74].Val;
  Val-=20;																// add malus for king on spawn
 }
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(Gm->color) Val=-Val;
 if(*flags==64) 								return Val;				// unclear position
 if(Gm->color) *flags=1; else *flags=2;									// white has at least pos value	
 return Val;
}

short 	KBvKBP(Game* Gm, Byte *flags)
{
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap BP=Gm->POSITION[1][6],BB=Gm->POSITION[1][4],WB=Gm->POSITION[0][4];// position of pawn and bishops
 Byte 	bps=find_b[(BP^BP-1)%67],bbs=find_b[(BB^BB-1)%67];				// square of pawn and bishop
 BitMap PS=PAWN_E[1][0][bps];											// pawn's spawn
 Byte 	wbs=find_b[(WB^WB-1)%67],pr=56+(bps&7);							// promotion square
 short 	Val;									
 
 *flags=128;															// default is back off
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers+					// more than one piece or pawn per side
 								(Gm->Count[1]).pawns>5) return 0;	
 if(((WB&WS)&&(BB&WS))||((WB&BS)&&(BB&BS))) 	 		return 0;		// bishops on same color
 Val=2*Square[1][5][1][bps]+20*Dist[wk][pr]-5*Dist[bk][pr];				// pawn's progress and distance of kings
 *flags=0;																// exact score
 if(WB&(STEP[0][bk]|STEP[11][bbs]|STEP[4][bps])) 	*flags=64; 			// bishop attacked
 if((BB|BP)&(STEP[1][wk]|STEP[11][wbs]))			*flags=64;			// bishop or pawn attacked
 if(BP&(P2|P3))										*flags=64;  		// pawn too far advanced	
 if(STEP[11][wbs]&PS) 													// bishop attacks pawn's spawn
 {
  if(!((BB&WB&BS)||(BB&WB&WS)))					return Paras[74].Val;	// bishops on different colours
  Val-=15;																// malus for bishop controls spawn 			
 }
 if(Gm->POSITION[0][1]&PS)												// king is on pawn's spawn
 {
  if(Gm->POSITION[0][1]&WS) {if(BB&BS) 			return Paras[74].Val;}	// king on spawn and not on bishop color
  else if(BB&WS) 								return Paras[74].Val;
  Val-=20;																// add malus for king on spawn
 }
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(!Gm->color) Val=-Val;
 if(*flags==64) 								return Val;				// unclear position
 if(Gm->color) *flags=2; else *flags=1;									// white has at least pos value	
 return Val;
}

short 	KNPvKB(Game* Gm, Byte *flags)
{
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap WP=Gm->POSITION[0][6],BB=Gm->POSITION[1][4],WN=Gm->POSITION[0][5]; // position of pawn, knight and bishop
 Byte 	wps=find_b[(WP^WP-1)%67],wns=find_b[(WN^WN-1)%67];				// square of pawn and knight
 Byte 	bbs=find_b[(BB^BB-1)%67],pr=wps&7;								// bishop and promotion square 
 BitMap PS=PAWN_E[0][0][wps];											// pawn's spawn
 short 	Val;									
 
 *flags=128;
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers+					// more than one piece or pawn per side
 								(Gm->Count[0]).pawns>5) return 0;
 Val=2*Square[0][5][1][wps]+20*Dist[bk][pr]-5*Dist[wk][pr];				// pawn's progress and distance of kings
 if(Gm->POSITION[1][1]&PS) 	Val-=30;									// black king is on pawn's spawn
 if(STEP[11][bbs]&PS)		Val-=40; 									// black bishop attacks black pawn's spawn
 Val=Val<Paras[74].Val?Paras[74].Val:Val;
 if(Gm->color) {*flags=1; Val=-Val;} else *flags=2;						// white has at least pos value	
 if(WP&(P6|P7))								 	 		*flags=64;		// pawn too far advanced
 if(BB&(STEP[1][wk]|STEP[2][wns]|STEP[3][wps])) 		*flags=64;		// bishop attacked
 if((WN|WP)&(STEP[1][bk]|STEP[11][bbs]))				*flags=64;		// knight or pawn attacked
 return Val;
}

short 	KBvKNP(Game* Gm, Byte *flags)
{
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap BP=Gm->POSITION[1][6],WB=Gm->POSITION[0][4],BN=Gm->POSITION[1][5]; // position of pawn, knight and bishop
 Byte 	bps=find_b[(BP^BP-1)%67],bns=find_b[(BN^BN-1)%67];				// square of pawn and knight
 Byte 	wbs=find_b[(WB^WB-1)%67],pr=56+(bps&7);							// bishop and promotion square 
 BitMap PS=PAWN_E[1][0][bps];											// pawn's spawn
 short 	Val;									
 
 *flags=128;
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers+					// more than one piece or pawn per side
 								(Gm->Count[1]).pawns>5) return 0;
 Val=2*Square[1][5][1][bps]+20*Dist[wk][pr]-5*Dist[bk][pr];				// pawn's progress and distance of kings
 if(Gm->POSITION[0][1]&PS) 	Val-=30;									// black king is on pawn's spawn
 if(STEP[11][wbs]&PS)		Val-=40; 									// black bishop attacks black pawn's spawn
 Val=Val<Paras[74].Val?Paras[74].Val:Val;
 if(Gm->color) *flags=2; else {*flags=1; Val=-Val;}						// white has at least pos value	
 if(BP&(P2|P3))								 	 		*flags=64;		// pawn too far advanced
 if(WB&(STEP[1][bk]|STEP[2][bns]|STEP[4][bps])) 		*flags=64;		// bishop attacked
 if((BN|BP)&(STEP[1][wk]|STEP[11][wbs]))				*flags=64;		// knight or pawn attacked
 return Val;
}

short 	KRvKB(Game* Gm, Byte *flags)
{
 BitMap BB=Gm->POSITION[1][4],WR=Gm->POSITION[0][3];					// position of bishop(s) and rook
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte 	wrs=find_b[(WR^WR-1)%67],bbs=find_b[(BB^BB-1)%67];				// rook square
 short 	Val;
 
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4) 
 											{*flags=128; return 0;}		// more than one rook or bishop
 if(BB&WS) Val=70-10*Dist_b[0][bk]; 									// distance of bK to bishop's color corner
 else Val=70-10*Dist_b[1][bk];	
 	
 Val-=5*Dist[wk][bk];													// distance between kings
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 if(Gm->color) {*flags=1; Val=-Val;} else 	*flags=2;					// black has at most pos val, maybe lost
 if(BB&(STEP[1][wk]|STEP[10][wrs])) 		*flags=64;					// bishop is attacked
 if(WR&(STEP[1][bk]|STEP[11][bbs])) 		*flags=64;					// rook is attacked
 return Val;
}

short 	KBvKR(Game* Gm, Byte *flags)
{
 BitMap WB=Gm->POSITION[0][4],BR=Gm->POSITION[1][3];					// position of bishop(s) and rook
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte 	brs=find_b[(BR^BR-1)%67],wbs=find_b[(WB^WB-1)%67];				// rook square
 short 	Val;
 
 if((Gm->Count[1]).officers+(Gm->Count[0]).officers>4) 
 											{*flags=128; return 0;}		// more than one rook or bishop
 if(WB&WS) Val=70-10*Dist_b[0][wk]; 									// distance of wK to bishop's color corner
 else Val=70-10*Dist_b[1][wk];	
 Val-=5*Dist[wk][bk];													// distance between kings
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// black has at least a draw
 if(Gm->color) *flags=2; else {*flags=1; Val=-Val;}						// white has at most pos val, maybe lost
 if(WB&(STEP[1][bk]|STEP[10][brs])) 		*flags=64;					// bishop is attacked
 if(BR&(STEP[1][wk]|STEP[11][wbs])) 		*flags=64;					// rook is attacked
 return Val;	
}

short 	KRvKN(Game* Gm, Byte *flags)
{
 BitMap BN=Gm->POSITION[1][5],WR=Gm->POSITION[0][3];					// position of knight(s) and rook
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte 	bns,wrs=find_b[(WR^WR-1)%67];									// rook square
 short 	Val;

 if(((Gm->Count[0]).officers>2)||((Gm->Count[1]).officers>3)) 			// more than one rook or two knights
  									{*flags=128; return 0;}
 if(Gm->color) *flags=1; else 					*flags=2;				// black has at most pos value
 Val=50+10*Dist_c[bk]-5*Dist[wk][bk];									// distance of king to center and king
 if(BN&(STEP[1][wk]|STEP[10][wrs]))				*flags=64;				// knight is attacked
 else if(WR&STEP[1][bk])						*flags=64;				// rook is attacked	
 while(BN) 
 {
  bns=find_b[(BN^BN-1)%67]; BN&=BN-1;									// knight square, next knight
  Val+=5*Dist[bns][bk]+10*Dist_c[bns]-50; 								// distance of knight to king and center
  if(WR&STEP[2][bns])							*flags=64;  			// rook is attacked
 }									
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// white has at least a draw
 
 if(Gm->color) return -Val; else return Val;
}

short 	KNvKR(Game* Gm, Byte *flags)
{
 BitMap WN=Gm->POSITION[0][5],BR=Gm->POSITION[1][3];					// position of knight(s) and rook
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 Byte 	wns,brs=find_b[(BR^BR-1)%67];									// rook square
 short 	Val;
 
 if(((Gm->Count[0]).officers>3)||((Gm->Count[1]).officers>2)) 			// more than one rook or two knights
  									{*flags=128; return 0;}
 if(Gm->color) *flags=2; else 					*flags=1;				// white has at most pos value
 Val=50+10*Dist_c[wk]-5*Dist[wk][bk];									// distance of king to center and king
 if(WN&(STEP[1][bk]|STEP[10][brs]))				*flags=64;				// knight is attacked
 else if(BR&(STEP[1][wk]))						*flags=64;				// rook is attacked	
 while(WN) 
 {
  wns=find_b[(WN^WN-1)%67]; WN&=WN-1;									// knight square, next knight
  Val+=5*Dist[wns][wk]+10*Dist_c[wns]-50; 								// distance of knight to king and center
  if(BR&STEP[2][wns])							*flags=64;  			// rook is attacked
 }									
 Val=Val<Paras[74].Val?Paras[74].Val:Val;								// black has at least a draw
 
 if(Gm->color) return Val; else return -Val;
}

short 	KBvKN(Game* Gm, Byte *flags)
{
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4)	*flags=128;		// one side has more than one piece
 else if((Gm->POSITION[0][1]|Gm->POSITION[1][1])&COR)	*flags=64;		// king in corner
 else													*flags=0; 		// exact score
 return Paras[74].Val-1;												// position is draw	
}

short 	KNvKB(Game* Gm, Byte *flags)
{
 if((Gm->Count[0]).officers+(Gm->Count[1]).officers>4)	*flags=128;		// one side has more than one piece
 else if((Gm->POSITION[0][1]|Gm->POSITION[1][1])&COR)	*flags=64;		// king in corner
 else													*flags=0; 		// exact score
 return Paras[74].Val-1;												// position is draw	
}

short 	KRvKP(Game* Gm, Byte *flags)
{
 *flags=128; return 0;
}

short 	KPvKR(Game* Gm, Byte *flags)
{
 *flags=128; return 0;
}

short 	KRPvKR(Game* Gm, Byte *flags)
{
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap WP=Gm->POSITION[0][6],BKB=Gm->POSITION[1][1],PS;				// position of pawn(s) and black king
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3];					// position of rooks
 Byte 	sp,r,lp,lk,ap=63,brb=find_b[(BR^BR-1)%67];						// square of foremost pawn and black rook
 short 	Val;
 
 if(((Gm->Count[0]).officers+(Gm->Count[1]).officers>4)||				// more than two rooks or pawns in game
  ((Gm->Count[0]).pawns>2))				{*flags=128; return 0;}			// back off
 *flags=64;	Val=(Gm->Count[0]).pawns*Paras[9].Val; PS=WP; lk=bk&7;		// default is unclear, material value
 while(PS)																// parse white pawns
 {
  sp=find_b[(PS^PS-1)%67]; PS&=PS-1; r=7-(sp>>3); lp=sp&7;				// rank of pawn
  if(sp<ap) ap=sp;														// square of most advanced pawn
  Val+=5*r*r+10*Dist[bk][sp&7]+2*(Dist[bk][sp]-Dist[wk][sp]);			// progress of pawn and distance from promotion
  if(BKB&PAWN_E[1][8][sp])  		Val-=10;							// black king on short pawn side
  if(WR&PAWN_E[0][8][sp])   		Val+=10;							// white rook on long pawn side
  if(((sp>>3)==1)&&(WR&P4)) 		Val+=20;							// Lucena (building a bridge)
  if(lk>lp) Val+=10*(lk-lp); else 	Val+=10*(lp-lk);					// black king cut off
 }
 if((ap>>3)<3)															// Philidor: pawn on rank>5
 {
  if(BR&P1) Val-=20; if(BR&P2) 		Val-=15; 							// black rook on rank 1 or 2
  if(BR&PAWN_E[0][8][ap]) 			Val-=10;							// black rook on long side
 }
 else if(((wk>>3)>3)&&(BR&P6)) 		Val-=20;							// white king < 6th rank
 Val-=Dist[wk][brb];													// distance of black rook to white king
 if(PAWN_E[0][0][brb]&WP) 			Val-=10;							// Tarrasch (attack pawn from behind)
 if(Gm->color) return -Val; else return Val;							// value depends on color
}

short 	KRvKRP(Game* Gm, Byte *flags)
{
 Byte   bk=(Gm->Officer[1][0]).square,wk=(Gm->Officer[0][0]).square;	// position of kings
 BitMap BP=Gm->POSITION[1][6],WKB=Gm->POSITION[0][1],PS;				// position of pawn(s) and white king
 BitMap WR=Gm->POSITION[0][3],BR=Gm->POSITION[1][3];					// position of rooks
 Byte 	sp,r,lp,lk,ap=0,wrb=find_b[(WR^WR-1)%67];						// square of foremost pawn and white rook
 short 	Val;
 
 if(((Gm->Count[0]).officers+(Gm->Count[1]).officers>4)||				// more than two rooks or pawns in game
  ((Gm->Count[1]).pawns>2))				{*flags=128; return 0;}			// back off
 *flags=64;	Val=(Gm->Count[1]).pawns*Paras[9].Val; PS=BP; lk=wk&7;		// default is unclear, material value
 while(PS)																// parse white pawns
 {
  sp=find_b[(PS^PS-1)%67]; PS&=PS-1; r=sp>>3; lp=sp&7;					// rank of pawn
  if(sp>ap) ap=sp;														// square of most advanced pawn
  Val+=5*r*r+10*Dist[wk][56+sp&7]+2*(Dist[wk][sp]-Dist[bk][sp]);		// progress of pawn and distance from promotion
  if(WKB&PAWN_E[1][8][sp]) 			Val-=10;							// white king on short pawn side
  if(BR&PAWN_E[0][8][sp]) 			Val+=10;							// black rook on long pawn side
  if(((sp>>3)==6)&&(BR&P5)) 		Val+=20;							// Lucena (building a bridge)
  if(lk>lp) Val+=10*(lk-lp); else 	Val+=10*(lp-lk);					// white king cut off
 }
 if((ap>>3)>4)															// Philidor: pawn on rank<4
 {
  if(WR&P8) Val-=20; if(WR&P7) 		Val-=15; 							// white rook on rank 7 or 8
  if(WR&PAWN_E[0][8][ap]) 			Val-=10;							// white rook on long side
 }
 else if(((bk>>3)<4)&&(WR&P3)) 		Val-=20;							// black king > 3rd rank
 Val-=Dist[bk][wrb];													// distance of white rook to black king
 if(PAWN_E[1][0][wrb]&BP) 			Val-=10;							// Tarrasch (attack pawn from behind)
 if(Gm->color) return Val; else return -Val;							// value depends on color
}

// Todo:
// KQvKR Endgame (700)
