#ifndef WORHP_MACROS_CORE_F
#define WORHP_MACROS_CORE_F

! check for NaN
#define ISNAN(X) (X/=X)

! macros for checking bound types
#define X_EQ(i)        (ABS(XU(i)-XL(i)) <= par%eps)
#define X_IEQ(i)       (.NOT. X_EQ(i) .AND. (XL(i) > -par%Infty .OR. XU(i) < par%Infty)
#define X_IEQ_LOW(i)   (.NOT. X_EQ(i) .AND. XL(i) > -par%Infty .AND. XU(i) >= par%Infty)
#define X_IEQ_UP(i)    (.NOT. X_EQ(i) .AND. XL(i) <= -par%Infty .AND. XU(i) < par%Infty)
#define X_IEQ_LOWUP(i) (.NOT. X_EQ(i) .AND. XL(i) > -par%Infty .AND. XU(i) < par%Infty)
#define G_EQ(i)        (ABS(GU(i)-GL(i)) <= par%eps)
#define G_IEQ(i)       (.NOT. G_EQ(i) .AND. (GL(i) > -par%Infty .OR. GU(i) < par%Infty)
#define G_IEQ_LOW(i)   (.NOT. G_EQ(i) .AND. GL(i) > -par%Infty .AND. GU(i) >= par%Infty)
#define G_IEQ_UP(i)    (.NOT. G_EQ(i) .AND. GL(i) <= -par%Infty .AND. GU(i) < par%Infty)
#define G_IEQ_LOWUP(i) (.NOT. G_EQ(i) .AND. GL(i) > -par%Infty .AND. GU(i) < par%Infty)

! macros for loops over equality or (lower / upper) inequality constraints
! Example for indices k, i, l and j. Usage is always the same. Index l just 
! appears for X, not for G.
! k: index in {1,...,n}
! l: index in {1,...,n-nXeq}
! i: index in {1,...,nXieq}
! j: index in {1,...,nXieqLow}
#define LOOP_X_EQ(k,i) i = 1, work%nXeq ;\
    k = idxXeq(i)
#define LOOP_X_NEQ(k,i) i = 1, work%nXneq ;\
    k = idxXneq(i)
#define LOOP_X_IEQ(k,l,i) i = 1, work%nXieq ;\
    l = idxXieq(i) ;\
    k = idxXneq(l)
#define LOOP_X_IEQ_LOW(k,l,i,j) j = 1, work%nXieqLowUp + work%nXieqLow ;\
    i = idxXieqLow(j) ;\
    l = idxXieq(i) ;\
    k = idxXneq(l)
#define LOOP_X_IEQ_UP(k,l,i,j) j = 1, work%nXieqLowUp + work%nXieqUp ;\
    i = idxXieqUp(j) ;\
    l = idxXieq(i) ;\
    k = idxXneq(l)
#define LOOP_X_IEQ_JUST_LOW(k,l,i,j) j = 1, work%nXieqLow ;\
    i = idxXieqLow(work%nXieqLowUp + j) ;\
    l = idxXieq(i) ;\
    k = idxXneq(l)
#define LOOP_X_IEQ_JUST_UP(k,l,i,j) j = 1, work%nXieqUp ;\
    i = idxXieqUp(work%nXieqLowUp + j) ;\
    l = idxXieq(i) ;\
    k = idxXneq(l)
#define LOOP_X_IEQ_JUST_LOWUP(k,l,i,j) j = 1, work%nXieqLowUp ;\
    i = idxXieqLow(j) ;\
    l = idxXieq(i) ;\
    k = idxXneq(l)
#define LOOP_G_EQ(k,i) i = 1, work%nGeq ;\
    k = idxGeq(i)
#define LOOP_G_IEQ(k,i) i = 1, work%nGieq ;\
    k = idxGieq(i)
#define LOOP_G_IEQ_LOW(k,i,j) j = 1, work%nGieqLowUp + work%nGieqLow ;\
    i = idxGieqLow(j) ;\
    k = idxGieq(i)
#define LOOP_G_IEQ_UP(k,i,j) j = 1, work%nGieqLowUp + work%nGieqUp ;\
    i = idxGieqUp(j) ;\
    k = idxGieq(i)
#define LOOP_G_IEQ_JUST_LOW(k,i,j) j = work%nGieqLowUp + 1, work%nGieqLowUp + work%nGieqLow ;\
    i = idxGieqLow(j) ;\
    k = idxGieq(i)
#define LOOP_G_IEQ_JUST_UP(k,i,j) j = work%nGieqLowUp + 1, work%nGieqLowUp + work%nGieqUp ;\
    i = idxGieqUp(j) ;\
    k = idxGieq(i)
#define LOOP_G_IEQ_JUST_LOWUP(k,i,j) j = 1, work%nGieqLowUp ;\
    i = idxGieqLow(j) ;\
    k = idxGieq(i)


! macros for loops over nonzeros of HM/DG without Xeq
#define LOOP_DF_XNEQ(k,i) i = 1, work%nnzDFXneq ;\
	k = idxDFXneq(i)
#define LOOP_DF_XNEQ_CS(k,i,row) i = 1, work%nnzDFXneq ;\
	k = idxDFXneq(i) ;\
	row = idxXneqInv(DFrow(k))
#define LOOP_HM_XNEQ(k,i) i = 1, work%nnzHMXneq ;\
    k = idxHMXneq(i)
#define LOOP_HM_XNEQ_CS(k,i,row,col) i = 1, work%nnzHMXneq ;\
    k = idxHMXneq(i) ;\
    row = idxXneqInv(HMrow(k)) ;\
    col = idxXneqInv(HMcol(k))
#define LOOP_DG_XNEQ(k,i) i = 1, work%nnzDGXneq ;\
    k = idxDGXneq(i)
#define LOOP_DG_XNEQ_CS(k,i,row,col) i = 1, work%nnzDGXneq ;\
    k = idxDGXneq(i) ;\
    row = DGrow(k) ;\
    col = idxXneqInv(DGcol(k))


#endif