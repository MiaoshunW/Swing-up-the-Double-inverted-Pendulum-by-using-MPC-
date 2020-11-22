#ifndef defines_h
#define defines_h

/** Erzwinge Kompatibilität unter Win zwischen Debug- und Release-Versionen */
//#define _ITERATOR_DEBUG_LEVEL 0
//#define _HAS_ITERATOR_DEBUGGING 0

/** Für Verwendung mit WORHP Zen kompilieren? */

//#ifdef WIN32
#define USE_ZEN
//#endif

/** Besser über Compiler-Aufruf einstellen: */
//#define NOGRAPHICS




enum DiscretizationType_e {TW_Trapez=0, TW_HermiteSimpson=1, TW_Euler=2, TW_Lobatto=3, TW_MultipleShooting=10};



#endif
