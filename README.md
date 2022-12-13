# Astimate
Astimate 3 UCI Engine by Dr. Axel Steinhage 2022
================================================

Contents:
---------

Astimate3.cpp:			Source code of Astimate3 in ANSI c

Astimate.h:			Header file with constant definitions required for compiling Astimate3.cpp

Astimate.pos:			Opening book containing around 22k positions generated from gm2600.bin

Astimate.tre:			Opening book containing around 400k positions

network.nnue:			NNUE-network (halfka_v2)


note: the opening book files and the header file are only required when compiling the source code on your own. At compile time, both books and the header constants are integrated in the executable so that in the engines-directory (e.g. of Arena) only the executable and network.nnue is required. Both books detect move transpositions. If the NNUE option is selected, the network.nnue file should be present in the same directory as the executable.

Compiling:
----------

The executables are ready to use in an UCI-capable chess-GUI in Windows (e.g. Arena http://www.playwitharena.de). However, if you need to compile the engine for your system, you need a the files Astimate3.cpp, Astimate.h, Astimate.pos and Astimate.tre and a c compiler.

Compiling under Windows (example): Use for instance DevC++ (https://bloodshed-dev-c.de.softonic.com), set Compiler Options (under "Tools") to 64 or 32 bit Release, set the "Optimization Level" to "Highest" and "Language Standard ISO C++11" (under "Tools->Compiler Options->Settings->Code Generation"), select general compiler option "-mavx2" (if machine has avx2) and compile ("Execute->Compile"). If machine has no avx, you need to comment out the line "#define USE_AVX2  1" near the beginning of the code.

Compiling under Linux (example): use "gcc Astimate3.cpp -Ofast -pthread -mavx2 -lpthread -o Astimate3". Astimate3 works on the DGT Pi too!

Compiling under Android (example): use the App "C4droid" and therein the compiler "GCC + Bionic" and export the binary (under Export). For Android you can use the app "DroidFish" as GUI.

I have not yet integrated NEON acceleration for ARM chips. Therefore, when NNUE is activated, slow generic neural network inference algorithms will be used.

Configuring the Engine:
-----------------------

Somne chess GUIs allow for a configuration of the engine. Astimate3 offers a huge number of configurable parameters. In Arena, for instance, you can go to "Engines->Configuration" to change parameters. However, the default parameters are tuned to give maximum strength.
Astimate3 is a multi-thread engine. The parameter "HelperThreads" controls, how many processor threads are used by Astimate3 in addition to the main thread. The more helper threads, the faster Astimate3 calculates. However, if more threads are configured than the computer can handle, the engine becomes even slower. The best setting depends, therefore, on your patform. The default is 3 which is a safe setting for most current processors.

Features of Astimate3:
----------------------

Openings:		Integrated opening books (450k positions) with transposition and setup position detection.

Endgame:		All elementary endgames (also KBNvK), integrated complete library for KPvK and KPvKP with blocked pawns

Search:		Bitboards, NegaScout, Principle Variation Search, Nullmove, Multicut, Internal Iterative Deepening, Late Move Reductions, 
			Delta Pruning, Quiescence Search, Mate Distance Pruning, Tactics Extensions, Static Exchange Evaluation, Counter and History heuristics, Pondering, ...
			
Hash Tables:	Two Tier Transposition Table, Evaluation Table, Pawn Table, Material Table

Multi Threads:	SMP (Shared Memory Parallelization) where the independent threads share history- and transposition table

NNUE:			Astimate can make use of "Efficently Updatable Neural Networks", coded in the file network.nnue. It supports the NNUE versions "halfkp" and "halfka_v2".

Copyright:
----------
Ask the author (axel.steinhage@web.de) before distributing exacutable or source code.

