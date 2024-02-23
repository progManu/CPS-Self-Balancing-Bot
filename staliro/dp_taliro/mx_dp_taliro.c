/***** mx_dp_taliro : mx_dp_taliro.c *****/

/* Written by Georgios Fainekos, ASU, U.S.A. for fw_taliro                */
/* Modified by Hengyi Yang, Adel Dokhanchi ASU, U.S.A. for dp_taliro      */
/* Copyright (c) 2011  Georgios Fainekos								  */
/* Send bug-reports and/or questions to: fainekos@gmail.com			      */

/* This program is free software; you can redistribute it and/or modify   */
/* it under the terms of the GNU General Public License as published by   */
/* the Free Software Foundation; either version 2 of the License, or      */
/* (at your option) any later version.                                    */
/*                                                                        */
/* This program is distributed in the hope that it will be useful,        */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of         */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          */
/* GNU General Public License for more details.                           */
/*                                                                        */
/* You should have received a copy of the GNU General Public License      */
/* along with this program; if not, write to the Free Software            */
/* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA*/

/* Some of the code in this file was taken from LTL2BA software           */
/* Written by Denis Oddoux, LIAFA, France					              */
/* Some of the code in this file was taken from the Spin software         */
/* Written by Gerard J. Holzmann, Bell Laboratories, U.S.A.               */


#include <stdlib.h>
#include "mex.h"
#include "matrix.h"
#include "distances.h"
#include "ltl2tree.h"
#include "param.h"


#define BUFF_LEN 4096

char * emalloc(size_t n)
{       
	char *tmp;
	
	if (!(tmp = (char *) mxMalloc(n)))
        mexErrMsgTxt("mx_dp_taliro: not enough memory!");
	memset(tmp, 0, n);
	return tmp;
}

int tl_Getchar(int *cnt, size_t hasuform, char *uform)
{
	if (*cnt < hasuform)
		return uform[(*cnt)++];
	(*cnt)++;
	return -1;
}

void tl_UnGetchar(int *cnt)
{
	if (*cnt > 0) (*cnt)--;
}

#define Binop(a)		\
		fprintf(miscell->tl_out, "(");	\
		dump(n->lft, miscell);		\
		fprintf(miscell->tl_out, a);	\
		dump(n->rgt, miscell);		\
		fprintf(miscell->tl_out, ")")

static void sdump(Node *n, char	*dumpbuf)
{
	switch (n->ntyp) {
	case PREDICATE:	strcat(dumpbuf, n->sym->name);
			break;
	case U_OPER:	strcat(dumpbuf, "U");
			goto common2;
	case V_OPER:	strcat(dumpbuf, "V");
			goto common2;
	case OR:	strcat(dumpbuf, "|");
			goto common2;
	case AND:	strcat(dumpbuf, "&");
common2:		sdump(n->rgt,dumpbuf);
common1:		sdump(n->lft,dumpbuf);
			break;
	case NEXT:	strcat(dumpbuf, "X");
			goto common1;
	case NOT:	strcat(dumpbuf, "!");
			goto common1;
	case TRUE:	strcat(dumpbuf, "T");
			break;
	case FALSE:	strcat(dumpbuf, "F");
			break;
	default:	strcat(dumpbuf, "?");
			break;
	}
}

Symbol *DoDump(Node *n, char *dumpbuf, Miscellaneous *miscell)
{
	if (!n) return ZS;

	if (n->ntyp == PREDICATE)
		return n->sym;

	dumpbuf[0] = '\0';
	sdump(n,dumpbuf);
	return tl_lookup(dumpbuf, miscell);
}

void dump(Node *n, Miscellaneous *miscell)
{
	if (!n) return;

	switch(n->ntyp) {
	case OR:	Binop(" || "); break;
	case AND:	Binop(" && "); break;
	case U_OPER:	Binop(" U ");  break;
	case V_OPER:	Binop(" V ");  break;
	case NEXT:
		fprintf(miscell->tl_out, "X");
		fprintf(miscell->tl_out, " (");
		dump(n->lft, miscell);
		fprintf(miscell->tl_out, ")");
		break;
	case NOT:
		fprintf(miscell->tl_out, "!");
		fprintf(miscell->tl_out, " (");
		dump(n->lft, miscell);
		fprintf(miscell->tl_out, ")");
		break;
	case FALSE:
		fprintf(miscell->tl_out, "false");
		break;
	case TRUE:
		fprintf(miscell->tl_out, "true");
		break;
	case PREDICATE:
		fprintf(miscell->tl_out, "(%s)", n->sym->name);
		break;
	case -1:
		fprintf(miscell->tl_out, " D ");
		break;
	default:
		printf("Unknown token: ");
		tl_explain(n->ntyp);
		break;
	}
}

void tl_explain(int n)
{
	switch (n) {
	case ALWAYS:	printf("[]"); break;
	case EVENTUALLY: printf("<>"); break;
	case IMPLIES:	printf("->"); break;
	case EQUIV:	printf("<->"); break;
	case PREDICATE:	printf("predicate"); break;
	case OR:	printf("||"); break;
	case AND:	printf("&&"); break;
	case NOT:	printf("!"); break;
	case U_OPER:	printf("U"); break;
	case V_OPER:	printf("V"); break;
	case NEXT:	printf("X"); break;
	case TRUE:	printf("true"); break;
	case FALSE:	printf("false"); break;
	case ';':	printf("end of formula"); break;
	default:	printf("%c", n); break;
	}
}

static void non_fatal(char *s1, char *s2, int *cnt, char *uform, int *tl_yychar, Miscellaneous *miscell)
{
	int i;

	printf("TaLiRo: ");
	if (s2)
		printf(s1, s2);
	else
		printf(s1);
	if ((*tl_yychar) != -1 && (*tl_yychar) != 0)
	{	printf(", saw '");
		tl_explain((*tl_yychar));
		printf("'");
	}
	printf("\nTaLiRo: %s\n---------", uform);
	for (i = 0; i < (*cnt); i++)
		printf("-");
	printf("^\n");
	fflush(stdout);
	(miscell->tl_errs)++;
}

void
tl_yyerror(char *s1, int *cnt, char *uform, int *tl_yychar, Miscellaneous *miscell)
{
	Fatal(s1, (char *) 0, cnt, uform, tl_yychar, miscell);
}

void
Fatal(char *s1, char *s2, int *cnt, char *uform, int *tl_yychar, Miscellaneous *miscell)
{
  non_fatal(s1, s2, cnt, uform, tl_yychar, miscell);
  tl_exit(0);
}

void fatal(char *s1, char *s2, int *cnt, char *uform, int *tl_yychar, Miscellaneous *miscell)
{
        non_fatal(s1, s2, cnt, uform, tl_yychar, miscell);
		tl_exit(0);
}

void put_uform(char *uform, Miscellaneous *miscell)
{
	fprintf(miscell->tl_out, "%s", uform);
}

void tl_exit(int i)
{
	mexErrMsgTxt("mx_dp_taliro: unexpected error, tl_exit executed.");
}

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{	
    /* Variables needed to process the input */
	int status, pstatus;
    mwSize buflen, pbuflen;
	size_t NElems,NCells;
	mwSize ndimA, ndimb, ndimG, ndim, pdim,ndimP;
	const mwSize *dimsA, *dimsb, *dimsG, *dims, *pardims,*dimsP;
	mwIndex jstruct, iii, jjj, i1, j1, idx_j;
	mxArray *tmp,*tmp1,*tmp_cell;	
	/* Variables needed for monitor */
	Node *node;
	double *XTrace, *TStamps, *LTrace;
	DistCompData distData;
	PMap *predMap; 
	int ii,jj,kk,ll,objs,tempI;
	bool par_on;
    bool is_Multi_HA;
	bool initial_of_par;
	int npred, npar;

	static char	uform[BUFF_LEN];
	static size_t hasuform=0;
	static int *cnt;
	int temp = 0;
	Miscellaneous *miscell = (Miscellaneous *) emalloc(sizeof(Miscellaneous));
	int *tl_yychar = (int *) emalloc(sizeof(int));
	miscell->dp_taliro_param.LTL = 1; 
	miscell->dp_taliro_param.ConOnSamples = 0; 
	miscell->dp_taliro_param.SysDim = 0; 
	miscell->dp_taliro_param.nSamp = 0; 
	miscell->dp_taliro_param.nPred = 0; 
	miscell->dp_taliro_param.true_nPred = 0; 
	miscell->dp_taliro_param.tnLoc = 0; 
	miscell->dp_taliro_param.nInp = 0; 
    miscell->dp_taliro_param.nCLG = 0;
	miscell->tl_errs = 0;
	miscell->type_temp = 0;

	/* Reset cnt to 0:
		cnt is the counter that points to the next symbol in the formula
		to be processed. This is a static variable and it retains its 
		value between Matlab calls to mx_dp_taliro. */
	cnt = &temp;

	/* Other initializations */
	miscell->dp_taliro_param.nInp = nrhs;
	par_on = false;
	initial_of_par = false;
	npred = 0;
	npar= 0;
 
	/* Make sure the I/O are in the right form */
    if(nrhs < 3)
		mexErrMsgTxt("mx_dp_taliro: At least 3 inputs are required.");
    else if(nlhs > 1)
      mexErrMsgTxt("mx_dp_taliro: Too many output arguments.");
    else if(nrhs > 8)
      mexErrMsgTxt("mx_dp_taliro: Too many input arguments.");
    else if(!mxIsChar(prhs[0]))
      mexErrMsgTxt("mx_dp_taliro: 1st input must be a string with TL formula.");
    else if(!mxIsStruct(prhs[1]))
      mexErrMsgTxt("mx_dp_taliro: 2nd input must be a structure (predicate map).");
    else if(!mxIsDouble(prhs[2]))
      mexErrMsgTxt("mx_dp_taliro: 3rd input must be a numerical array (State trace).");
    else if(nrhs>3 && !mxIsDouble(prhs[3]))
      mexErrMsgTxt("mx_dp_taliro: 4th input must be a numerical array (Time stamps).");
    else if(nrhs>5 && !mxIsDouble(prhs[4]))
      mexErrMsgTxt("mx_dp_taliro: 5th input must be a numerical array (Location trace).");
    else if(nrhs>5 && !(mxIsDouble(prhs[5])||mxIsCell(prhs[5])))
      mexErrMsgTxt("mx_dp_taliro: 6th input must be a numerical array \n (Minimum path distance to each control location for each predicate).");
    else if(nrhs>7 && !(mxIsCell(prhs[6])))
      mexErrMsgTxt("mx_dp_taliro: 7th input must be a cell array (Adjacency list).");
    else if(nrhs>7 && !(mxIsStruct(prhs[7])||mxIsCell(prhs[7])))
      mexErrMsgTxt("mx_dp_taliro: 8th input must be a structure (guard map).");

	if(nlhs > 1)
		mexErrMsgTxt("Too many output arguments.");
	plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);

	/* Process inputs */

	/* Get the formula */
	ndim = mxGetNumberOfDimensions(prhs[0]);
	dims = mxGetDimensions(prhs[0]);
	buflen = dims[1]*sizeof(mxChar)+1;
	if (buflen >= BUFF_LEN)
	{
      mexPrintf("%s%d%s\n", "The formula must be less than ", BUFF_LEN," characters.");
      mexErrMsgTxt("mx_dp_taliro: Formula too long.");
	}
	status = mxGetString(prhs[0], uform, buflen);   
    hasuform = strlen(uform);
    for (iii=0; iii<hasuform; iii++)
	{
		if (uform[iii] == '\t' || uform[iii] == '\"' || uform[iii] == '\n')						
			uform[iii] = ' ';
	}

	/* Get state trace */	    
	ndim = mxGetNumberOfDimensions(prhs[2]);
	if (ndim>2)
		mexErrMsgTxt("mx_dp_taliro: The state trace is not in proper form!"); 
	dims = mxGetDimensions(prhs[2]);
	miscell->dp_taliro_param.nSamp = dims[0];
	miscell->dp_taliro_param.SysDim = dims[1];
	XTrace = mxGetPr(prhs[2]);

	/* Get time stamps */	   
	if (nrhs>3)
	{
		ndim = mxGetNumberOfDimensions(prhs[3]);
		if (ndim>2)
			mexErrMsgTxt("mx_dp_taliro: The time stamp sequence is not in proper form!"); 
		dims = mxGetDimensions(prhs[3]);
		if (miscell->dp_taliro_param.nSamp != dims[0])
			mexErrMsgTxt("mx_dp_taliro: The lengths of the time stamp sequence and the state trace do not match!"); 
		TStamps = mxGetPr(prhs[3]);
	}

	/* Get location trace and location graph */	   
	if (nrhs>4)
	{
		ndim = mxGetNumberOfDimensions(prhs[4]);
		if (ndim>2)
			mexErrMsgTxt("mx_dp_taliro: The location trace is not in proper form!"); 
		dims = mxGetDimensions(prhs[4]);
		if (miscell->dp_taliro_param.nSamp != dims[0])
			mexErrMsgTxt("mx_dp_taliro: The lengths of the location trace and the state trace do not match!"); 
		LTrace = mxGetPr(prhs[4]);

        /* For Multiple H.A.s is added  */
        if(nrhs>5 && (mxIsCell(prhs[5]))){/*  For Multiple H.A.s */
            is_Multi_HA=1;
        miscell->dp_taliro_param.nCLG = dims[1];
    		ndim = mxGetNumberOfDimensions(prhs[5]);
			if (ndim>2)
			{
				mexErrMsgTxt("mx_dp_taliro.c: !"); 
			}
			dims = mxGetDimensions(prhs[5]);
			if (dims[0]!=1)
            {
                mexErrMsgTxt("mx_dp_taliro: DMin is a cell, it must be a column vector cell!"); 
			}
			if (dims[1]!=miscell->dp_taliro_param.nCLG)
            {
                mexErrMsgTxt("mx_dp_taliro: DMin is a cell, it must be of the size of nCLG!"); 
			}
            distData.LDistNCLG = (double **)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(double*));
            miscell->dp_taliro_param.tnLocNCLG = (mwSize*)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(mwSize));
            for(kk=0; kk<miscell->dp_taliro_param.nCLG; kk++){
                tmp = mxGetCell(prhs[5],kk);
                dims = mxGetDimensions(tmp);
                miscell->dp_taliro_param.tnLocNCLG[kk] = dims[0];
                distData.LDistNCLG[kk]=mxGetPr(tmp);
            }
        }
        else if(nrhs>5) {/*  For Single H.A. */
            is_Multi_HA=0;
            ndim = mxGetNumberOfDimensions(prhs[5]);
            if (ndim>2)
                mexErrMsgTxt("mx_dp_taliro: The minimum distance array is not in proper form!"); 
            dims = mxGetDimensions(prhs[5]);
            miscell->dp_taliro_param.tnLoc = dims[0];
            distData.LDist = mxGetPr(prhs[5]);
        }
         
	}

	/* Get guards */
	if (nrhs>7)
	{
        if(is_Multi_HA==0){
        /*  ONE Hybrid Automata  */                  
        NElems = mxGetNumberOfElements(prhs[6]);
		if (NElems==0)
		{
			mexErrMsgTxt("mx_dp_taliro: the adjacency list is empty!");
		}
		distData.AdjL = (double **)emalloc(NElems*sizeof(double*));
		distData.AdjLNell = (size_t *)emalloc(NElems*sizeof(size_t));
		for (ii=0; ii<NElems; ii++)
		{
			distData.AdjL[ii] = mxGetPr(mxGetCell(prhs[6],ii));
			ndim = mxGetNumberOfDimensions(mxGetCell(prhs[6],ii));
			dims = mxGetDimensions(mxGetCell(prhs[6],ii));
			if (ndim>2 || dims[0]>1)
			{
				mexErrMsgTxt("mx_dp_taliro: The adjacency list is not in correct format!"); 
			}
			distData.AdjLNell[ii] = dims[1];
		}

		ndimG = mxGetNumberOfDimensions(prhs[7]);
		if (ndimG>2)
		{
			mexErrMsgTxt("mx_dp_taliro: The guard sets are not in proper form!"); 
		}
		dimsG = mxGetDimensions(prhs[7]);
		if ((dimsG[0] != dimsG[1]) || (dimsG[0] != miscell->dp_taliro_param.tnLoc))
		{
			mexErrMsgTxt("mx_dp_taliro: The guard array must be a square array structure or \n the dimensions of the guard array do not match the adjacency matrix!"); 
		}
		distData.GuardMap = (GuardSet **)emalloc(dimsG[0]*sizeof(GuardSet*));
		for (ii=0; ii<dimsG[0]; ii++)
		{
			if (distData.AdjLNell[ii]>0)
			{
				distData.GuardMap[ii] = (GuardSet *)emalloc(distData.AdjLNell[ii]*sizeof(GuardSet));
				for (jj=0; jj<distData.AdjLNell[ii]; jj++)
				{
					/* Get set for guard (ii,jj) */
					idx_j = ((mwIndex) distData.AdjL[ii][jj])-1;
                    /* for projection is added */
					tmp_cell = mxGetField(prhs[7], ii+idx_j*dimsG[0], "proj");
                	if(tmp_cell == NULL)
                   	{
                        distData.GuardMap[ii][jj].nproj=0;
                    }
					else
					{
						ndimP = mxGetNumberOfDimensions(tmp_cell);
						if (ndimP>2)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: The projection map is not in proper form!"); 
						}
						dimsP = mxGetDimensions(tmp_cell);
						if(dimsP[0]!=1)
                        {
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: The projection map must be a row vector !"); 
						}
						if(dimsP[1]>miscell->dp_taliro_param.SysDim)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: The projection map must have size less than or equal to the system dimentions !"); 
						}
						distData.GuardMap[ii][jj].nproj=dimsP[1];
						distData.GuardMap[ii][jj].proj=(int *)emalloc(distData.GuardMap[ii][jj].nproj*sizeof(int));
						tempI=0;
						for (iii=0; iii<distData.GuardMap[ii][jj].nproj; iii++)
						{
							distData.GuardMap[ii][jj].proj[iii] = (mxGetPr(tmp_cell))[iii];
							if(distData.GuardMap[ii][jj].proj[iii]>miscell->dp_taliro_param.SysDim)
							{
								mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
								mexErrMsgTxt("mx_dp_taliro: Above guard: The projection index must be less than or equal to the system dimentions !"); 
							}
							if(distData.GuardMap[ii][jj].proj[iii]<=tempI)
							{
								mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
								mexErrMsgTxt("mx_dp_taliro: Above guard: The projection index must be incremental !"); 
							}
							else
							{
								tempI=distData.GuardMap[ii][jj].proj[iii];
							}
						}
					}
                    /* for projection */
                    idx_j = ((mwIndex) distData.AdjL[ii][jj])-1;
					tmp_cell = mxGetField(prhs[7], ii+idx_j*dimsG[0], "A");
					if (tmp_cell == NULL)
					{
						mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
						mexErrMsgTxt("mx_dp_taliro: Above guard: Field 'A' is undefined!"); 
					}
					/* If it is a cell, then the guard set is a union of polytopes */
					if (mxIsCell(tmp_cell))
					{
						ndim = mxGetNumberOfDimensions(tmp_cell);
						if (ndim>2)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: if A is a cell, it must be a column vector cell!"); 
						}
						dims = mxGetDimensions(tmp_cell);
						if (dims[0]!=1)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: if A is a cell, it must be a column vector cell!"); 
						}
						distData.GuardMap[ii][jj].nset = dims[1]; /* the number of sets */
					}
					else
					{
						/* For backward combatibility, non-cell inputs should be also accepted */
						distData.GuardMap[ii][jj].nset = 1; /* the number of sets */
					}
					
					distData.GuardMap[ii][jj].ncon = (int *)emalloc(distData.GuardMap[ii][jj].nset*sizeof(int));
                    distData.GuardMap[ii][jj].nproj=0;
					distData.GuardMap[ii][jj].A = (double ***)emalloc(distData.GuardMap[ii][jj].nset*sizeof(double**));
					for (kk=0; kk<distData.GuardMap[ii][jj].nset; kk++)
					{
						if (mxIsCell(tmp_cell))
						{
							tmp = mxGetCell(tmp_cell,kk);
						}
						else
						{
							tmp = tmp_cell;
						}
						ndimA = mxGetNumberOfDimensions(tmp);
						if (ndimA>2)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: A is not in proper form!"); 
						}
						dimsA = mxGetDimensions(tmp);
						if (miscell->dp_taliro_param.SysDim != dimsA[1])
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: The dimensions of the set constraints and the state trace do not match!"); 
						}
						distData.GuardMap[ii][jj].ncon[kk] = dimsA[0]; /* the number of constraints */
						if (distData.GuardMap[ii][jj].ncon[kk]>2 && miscell->dp_taliro_param.SysDim==1)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: For 1D signals only up to two constraints per predicate are allowed!\n More than two are redundant!"); 
						}
						distData.GuardMap[ii][jj].A[kk] = (double **)emalloc(distData.GuardMap[ii][jj].ncon[kk]*sizeof(double*));
						for (i1=0; i1<distData.GuardMap[ii][jj].ncon[kk]; i1++)
						{
							distData.GuardMap[ii][jj].A[kk][i1] = (double *)emalloc(miscell->dp_taliro_param.SysDim*sizeof(double));
							for (j1=0; j1<miscell->dp_taliro_param.SysDim; j1++)
							{
								distData.GuardMap[ii][jj].A[kk][i1][j1] = (mxGetPr(tmp))[i1+j1*distData.GuardMap[ii][jj].ncon[kk]];
						
							}
						}
					}
					/* get b */
					tmp_cell = mxGetField(prhs[7], ii+idx_j*dimsG[0], "b");
					if (tmp_cell == NULL)
					{
						mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
						mexErrMsgTxt("mx_dp_taliro: Above guard: Field 'b' is undefined!"); 
					}
					/* If it is a cell, then the guard set is a union of polytopes */
					if (mxIsCell(tmp_cell))
					{
						ndim = mxGetNumberOfDimensions(tmp_cell);
						if (ndim>2)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: if b is a cell, it must be a column vector cell!"); 
						}
						dims = mxGetDimensions(tmp_cell);
						if (dims[0]!=1)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: if b is a cell, it must be a column vector cell!"); 
						}
						if (distData.GuardMap[ii][jj].nset!=dims[1])
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: the dimensions of 'A' and 'b' must match!"); 
						}
					}
					distData.GuardMap[ii][jj].b = (double **)emalloc(distData.GuardMap[ii][jj].nset*sizeof(double*));
					for (kk=0; kk<distData.GuardMap[ii][jj].nset; kk++)
					{
						if (mxIsCell(tmp_cell))
						{
							tmp = mxGetCell(tmp_cell,kk);
						}
						else
						{
							tmp = tmp_cell;
						}
						ndimb = mxGetNumberOfDimensions(tmp);
						if (ndimb>2)
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: The set constraints are not in proper form!"); 
						}
						dimsb = mxGetDimensions(tmp);
						if (distData.GuardMap[ii][jj].ncon[kk] != dimsb[0])
						{
							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
							mexErrMsgTxt("mx_dp_taliro: Above guard: The number of constraints between A and b do not match!"); 
						}
						distData.GuardMap[ii][jj].b[kk] = mxGetPr(tmp);
						if (distData.GuardMap[ii][jj].ncon[kk]==2 && miscell->dp_taliro_param.SysDim==1)
						{
							if ((distData.GuardMap[ii][jj].A[kk][0][0]>0 && distData.GuardMap[ii][jj].A[kk][1][0]>0) || 
								(distData.GuardMap[ii][jj].A[kk][0][0]<0 && distData.GuardMap[ii][jj].A[kk][1][0]<0))
							{
								mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
								mexErrMsgTxt("mx_dp_taliro: Above guard: The set has redundant constraints! Please remove redundant constraints."); 
							}
							if (!((distData.GuardMap[ii][jj].A[kk][0][0]<0 && (distData.GuardMap[ii][jj].b[kk][0]/distData.GuardMap[ii][jj].A[kk][0][0]<=distData.GuardMap[ii][jj].b[kk][1]/distData.GuardMap[ii][jj].A[kk][1][0])) || 
								(distData.GuardMap[ii][jj].A[kk][1][0]<0 && (distData.GuardMap[ii][jj].b[kk][1]/distData.GuardMap[ii][jj].A[kk][1][0]<=distData.GuardMap[ii][jj].b[kk][0]/distData.GuardMap[ii][jj].A[kk][0][0]))))
							{
								mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
								mexErrMsgTxt("mx_dp_taliro: Above guard: The set is empty! Please modify the constraints."); 
							}
						}
					}
				}
			}
		}
              /*  ONE Hybrid Automata  */        
      }
        else{
      /*  MULTIPLE Hybrid Automata  */        
            distData.GuardMapNCLG = (GuardSet ***)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(GuardSet **));
            distData.AdjLnCLG = (double ***)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(double  **));
            distData.AdjLNellNCLG = (size_t **)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(size_t *));
            for(objs=0;objs<miscell->dp_taliro_param.nCLG;objs++){
                tmp1 = mxGetCell(prhs[6],objs);
            	NElems = mxGetNumberOfElements(tmp1);
        		if (NElems==0)
                {
                mexErrMsgTxt("mx_dp_taliro: the adjacency list is empty!");
        		}
                distData.AdjLnCLG[objs] = (double **)emalloc(NElems*sizeof(double*));
        		distData.AdjLNellNCLG[objs] = (size_t *)emalloc(NElems*sizeof(size_t));
        		for (ii=0; ii<NElems; ii++)
                {
        			distData.AdjLnCLG[objs][ii] = mxGetPr(mxGetCell(tmp1,ii));
        			ndim = mxGetNumberOfDimensions(mxGetCell(tmp1,ii));
        			dims = mxGetDimensions(mxGetCell(tmp1,ii));
        			if (ndim>2 || dims[0]>1)
        			{
        				mexErrMsgTxt("mx_dp_taliro: The adjacency list is not in correct format!"); 
        			}
        			distData.AdjLNellNCLG[objs][ii] = dims[1];
        		}
                tmp1 = mxGetCell(prhs[7],objs);
         		ndimG = mxGetNumberOfDimensions(tmp1);
         		if (ndimG>2)
                {
         			mexErrMsgTxt("mx_dp_taliro: The guard sets are not in proper form!"); 
         		}
         		dimsG = mxGetDimensions(tmp1);
         		if ((dimsG[0] != dimsG[1]) || (dimsG[0] != miscell->dp_taliro_param.tnLocNCLG[objs]))
         		{
         			mexErrMsgTxt("mx_dp_taliro: The guard array must be a square array structure or \n the dimensions of the guard array do not match the adjacency matrix!"); 
         		}
         		distData.GuardMapNCLG[objs] = (GuardSet **)emalloc(dimsG[0]*sizeof(GuardSet*));
		 		for (ii=0; ii<dimsG[0]; ii++)
		 		{
		 			if (distData.AdjLNellNCLG[objs][ii]>0)
		 			{
		 				distData.GuardMapNCLG[objs][ii] = (GuardSet *)emalloc(distData.AdjLNellNCLG[objs][ii]*sizeof(GuardSet));
		 				for (jj=0; jj<distData.AdjLNellNCLG[objs][ii]; jj++)
		 				{
		 					/* Get set for guard (ii,jj) */
		 					idx_j = ((mwIndex) distData.AdjLnCLG[objs][ii][jj])-1;
                   			tmp_cell = mxGetField(tmp1, ii+idx_j*dimsG[0], "proj");
                			if(tmp_cell == NULL) 
                   			{
                                distData.GuardMapNCLG[objs][ii][jj].nproj=0;
                			}
							else
							{
								ndimP = mxGetNumberOfDimensions(tmp_cell);
								if (ndimP>2)
								{
									mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
									mexErrMsgTxt("mx_dp_taliro: Above guard: The projection map is not in proper form!"); 
								}
								dimsP = mxGetDimensions(tmp_cell);
								if(dimsP[0]!=1)
								{
									mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
									mexErrMsgTxt("mx_dp_taliro: Above guard: The projection map must be a row vector !"); 
								}
								if(dimsP[1]>miscell->dp_taliro_param.SysDim)
								{
									mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
									mexErrMsgTxt("mx_dp_taliro: Above guard: The projection map must have size less than or equal to the system dimentions !"); 
								}
								distData.GuardMapNCLG[objs][ii][jj].nproj=dimsP[1];
								distData.GuardMapNCLG[objs][ii][jj].proj=(int *)emalloc(distData.GuardMapNCLG[objs][ii][jj].nproj*sizeof(int));
								tempI=0;
								for (iii=0; iii<distData.GuardMapNCLG[objs][ii][jj].nproj; iii++)
								{
									distData.GuardMapNCLG[objs][ii][jj].proj[iii] = (mxGetPr(tmp_cell))[iii];
									if(distData.GuardMapNCLG[objs][ii][jj].proj[iii]>miscell->dp_taliro_param.SysDim)
									{
										mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
										mexErrMsgTxt("mx_dp_taliro: Above guard: The projection index must be less than or equal to the system dimentions !"); 
									}
									if(distData.GuardMapNCLG[objs][ii][jj].proj[iii]<=tempI)
									{
										mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
										mexErrMsgTxt("mx_dp_taliro: Above guard: The projection index must be incremental !"); 
									}
									else
									{
										tempI=distData.GuardMapNCLG[objs][ii][jj].proj[iii];
									}
								}
							}
		 					tmp_cell = mxGetField(tmp1, ii+idx_j*dimsG[0], "A");
		 					if (tmp_cell == NULL)
		 					{
		 						mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 						mexErrMsgTxt("mx_dp_taliro: Above guard: Field 'A' is undefined!"); 
		 					}
		 					/* If it is a cell, then the guard set is a union of polytopes */
		 					if (mxIsCell(tmp_cell))
		 					{
		 						ndim = mxGetNumberOfDimensions(tmp_cell);
		 						if (ndim>2)
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: if A is a cell, it must be a column vector cell!"); 
		 						}
		 						dims = mxGetDimensions(tmp_cell);
		 						if (dims[0]!=1)
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: if A is a cell, it must be a column vector cell!"); 
		 						}
		 						distData.GuardMapNCLG[objs][ii][jj].nset = dims[1]; /* the number of sets */
		 					}
		 					else
		 					{
		 						/* For backward combatibility, non-cell inputs should be also accepted */
		 						distData.GuardMapNCLG[objs][ii][jj].nset = 1; /* the number of sets */
		 					}
 					
		 					distData.GuardMapNCLG[objs][ii][jj].ncon = (int *)emalloc(distData.GuardMapNCLG[objs][ii][jj].nset*sizeof(int));
		 					distData.GuardMapNCLG[objs][ii][jj].A = (double ***)emalloc(distData.GuardMapNCLG[objs][ii][jj].nset*sizeof(double**));
		 					for (kk=0; kk<distData.GuardMapNCLG[objs][ii][jj].nset; kk++)
		 					{
		 						if (mxIsCell(tmp_cell))
		 						{
		 							tmp = mxGetCell(tmp_cell,kk);
		 						}
		 						else
		 						{
		 							tmp = tmp_cell;
		 						}
		 						ndimA = mxGetNumberOfDimensions(tmp);
		 						if (ndimA>2)
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: A is not in proper form!"); 
		 						}
		 						dimsA = mxGetDimensions(tmp);
		 						if ((miscell->dp_taliro_param.SysDim != dimsA[1]&&distData.GuardMapNCLG[objs][ii][jj].nproj==0)||(distData.GuardMapNCLG[objs][ii][jj].nproj!=dimsA[1]&&distData.GuardMapNCLG[objs][ii][jj].nproj>0))
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: The dimensions of the set constraints and the state trace do not match!");
		 						}
		 						distData.GuardMapNCLG[objs][ii][jj].ncon[kk] = dimsA[0]; /* the number of constraints */
		 						if (distData.GuardMapNCLG[objs][ii][jj].ncon[kk]>2 && miscell->dp_taliro_param.SysDim==1)
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: For 1D signals only up to two constraints per predicate are allowed!\n More than two are redundant!"); 
		 						}
		 						distData.GuardMapNCLG[objs][ii][jj].A[kk] = (double **)emalloc(distData.GuardMapNCLG[objs][ii][jj].ncon[kk]*sizeof(double*));
								if(distData.GuardMapNCLG[objs][ii][jj].nproj==0)
		 						{
			 						for (i1=0; i1<distData.GuardMapNCLG[objs][ii][jj].ncon[kk]; i1++)
									{
										distData.GuardMapNCLG[objs][ii][jj].A[kk][i1] = (double *)emalloc(miscell->dp_taliro_param.SysDim*sizeof(double));
		 								for (j1=0; j1<miscell->dp_taliro_param.SysDim; j1++)
		 								{
		 									distData.GuardMapNCLG[objs][ii][jj].A[kk][i1][j1] = (mxGetPr(tmp))[i1+j1*distData.GuardMapNCLG[objs][ii][jj].ncon[kk]];
		 								}
									}
		 						}
								else
								{
			 						for (i1=0; i1<distData.GuardMapNCLG[objs][ii][jj].ncon[kk]; i1++)
									{
										distData.GuardMapNCLG[objs][ii][jj].A[kk][i1] = (double *)emalloc(distData.GuardMapNCLG[objs][ii][jj].nproj*sizeof(double));
		 								for (j1=0; j1<distData.GuardMapNCLG[objs][ii][jj].nproj; j1++)
		 								{
		 									distData.GuardMapNCLG[objs][ii][jj].A[kk][i1][j1] = (mxGetPr(tmp))[i1+j1*distData.GuardMapNCLG[objs][ii][jj].ncon[kk]];
		 								}
									}
								}
		 					}
		 					/* get b */
		 					tmp_cell = mxGetField(tmp1, ii+idx_j*dimsG[0], "b");
		 					if (tmp_cell == NULL)
		 					{
		 						mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 						mexErrMsgTxt("mx_dp_taliro: Above guard: Field 'b' is undefined!"); 
		 					}
		 					/* If it is a cell, then the guard set is a union of polytopes */
		 					if (mxIsCell(tmp_cell))
		 					{
		 						ndim = mxGetNumberOfDimensions(tmp_cell);
		 						if (ndim>2)
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: if b is a cell, it must be a column vector cell!"); 
		 						}
		 						dims = mxGetDimensions(tmp_cell);
		 						if (dims[0]!=1)
		 						{
									mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: if b is a cell, it must be a column vector cell!"); 
		 						}
		 						if (distData.GuardMapNCLG[objs][ii][jj].nset!=dims[1])
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: the dimensions of 'A' and 'b' must match!"); 
		 						}
		 					}
		 					distData.GuardMapNCLG[objs][ii][jj].b = (double **)emalloc(distData.GuardMapNCLG[objs][ii][jj].nset*sizeof(double*));
		 					for (kk=0; kk<distData.GuardMapNCLG[objs][ii][jj].nset; kk++)
		 					{
		 						if (mxIsCell(tmp_cell))
		 						{
		 							tmp = mxGetCell(tmp_cell,kk);
		 						}
		 						else
		 						{
		 							tmp = tmp_cell;
		 						}
		 						ndimb = mxGetNumberOfDimensions(tmp);
		 						if (ndimb>2)
		 						{
		 							mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: The set constraints are not in proper form!"); 
		 						}
		 						dimsb = mxGetDimensions(tmp);
		 						if (distData.GuardMapNCLG[objs][ii][jj].ncon[kk] != dimsb[0])
		 						{
		 							mexPrintf("%s%d%s%d%s%d%s \n","CLG(",objs+1 ,")Guard (",ii+1,",",idx_j+1,")");
		 							mexErrMsgTxt("mx_dp_taliro: Above guard: The number of constraints between A and b do not match!"); 
		 						}
		 						distData.GuardMapNCLG[objs][ii][jj].b[kk] = mxGetPr(tmp);
		 						if (distData.GuardMapNCLG[objs][ii][jj].ncon[kk]==2 && miscell->dp_taliro_param.SysDim==1)
		 						{
		 							if ((distData.GuardMapNCLG[objs][ii][jj].A[kk][0][0]>0 && distData.GuardMapNCLG[objs][ii][jj].A[kk][1][0]>0) || 
		 								(distData.GuardMapNCLG[objs][ii][jj].A[kk][0][0]<0 && distData.GuardMapNCLG[objs][ii][jj].A[kk][1][0]<0) ||
		 								!((distData.GuardMapNCLG[objs][ii][jj].A[kk][0][0]<0 && (distData.GuardMapNCLG[objs][ii][jj].A[kk][0][0]/distData.GuardMapNCLG[objs][ii][jj].b[kk][0]<=distData.GuardMapNCLG[objs][ii][jj].A[kk][1][0]/distData.GuardMapNCLG[objs][ii][jj].b[kk][1])) || 
		 								(distData.GuardMapNCLG[objs][ii][jj].A[kk][1][0]<0 && (distData.GuardMapNCLG[objs][ii][jj].A[kk][1][0]/distData.GuardMapNCLG[objs][ii][jj].b[kk][1]<=distData.GuardMapNCLG[objs][ii][jj].A[kk][0][0]/distData.GuardMapNCLG[objs][ii][jj].b[kk][0]))))
		 							{
		 								mexPrintf("%s%d%s%d%s \n", "Guard (",ii+1,",",idx_j+1,")");
		 								mexErrMsgTxt("mx_dp_taliro: Above guard: The constraint is the empty set!"); 
		 							}
		 						}
		 					}
		 				}
		 			}
		 		}
    
            }
      /*  MULTIPLE Hybrid Automata  */        
        }
	}

	/* Get predicate map*/
    NElems = mxGetNumberOfElements(prhs[1]);
	miscell->dp_taliro_param.nPred = NElems;
	miscell->dp_taliro_param.true_nPred = NElems;
    if (NElems==0)
        mexErrMsgTxt("mx_dp_taliro: the predicate map is empty!");
    predMap = (PMap *)emalloc(NElems*sizeof(PMap));
	miscell->parMap = (ParMap *)emalloc(NElems*sizeof(ParMap));
	miscell->predMap = (PMap *)emalloc(NElems*sizeof(PMap));

    miscell->pList.pindex=(int *)emalloc(NElems*sizeof(int));

    /* initial predicate list*/
	for(ll = 0; ll < NElems; ll++)
	{
		miscell->pList.pindex[ll] = -1;
	}

	for(jstruct = 0; jstruct < NElems; jstruct++) 
	{
        /* Get predicate name */
        tmp = mxGetField(prhs[1], jstruct, "str");
		if(tmp == NULL)
		{
			tmp = mxGetField(prhs[1], jstruct, "par");
			if(tmp == NULL) 
			{
				mexPrintf("%s%d\n", "Predicate no ", jstruct+1);
				mexErrMsgTxt("mx_dp_taliro: The above parameter must has either the 'str' field or 'par' field!");
			}
			else
			{	
				par_on = true;
				npar++;
			}
		}
		else
		{
			par_on = false;
			npred++;
		}

		/* If this is a parameter */
		if(par_on)
		{
			if(!initial_of_par)
			{
				miscell->dp_taliro_param.true_nPred = jstruct;
				initial_of_par = true;
			}
			/* Get name of the parameter */
			pdim = mxGetNumberOfDimensions(tmp);
			pardims = mxGetDimensions(tmp);
			
			pbuflen = pardims[1]*sizeof(mxChar)+1;
			miscell->parMap[jstruct].str = (char *)emalloc(pbuflen);   
			miscell->parMap[jstruct].index = (int) jstruct;   
			predMap[jstruct].true_pred = false;
			pstatus = mxGetString(tmp, miscell->parMap[jstruct].str, pbuflen);
		
			/* Get value */
			tmp = mxGetField(prhs[1], jstruct, "value");
			if(tmp == NULL) /* TODO */
			{
				tmp = mxGetField(prhs[1], jstruct, "range");
				if(tmp == NULL)
				{
					mexPrintf("%s%s \n", "Predicate: ", miscell->parMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: both 'value' and 'range' do not exist!"); 
				}
			}
			else if(mxIsEmpty(tmp))
			{ 
				mexPrintf("%s%s \n", "Predicate: ", miscell->parMap[jstruct].str);
				mexErrMsgTxt("mx_dp_taliro: Above predicate: 'value' is empty when 'par' exist which is not allowed !"); 
			}
			else
			{
				miscell->parMap[jstruct].value = mxGetPr(tmp);
			}

			/* Get range*/
			tmp = mxGetField(prhs[1], jstruct, "range");
			if(tmp != NULL)
			{
				/* TODO: This should be moved when reading field b in order to check if dimensions match */
				/* Error checks that the bounds are correct? Should this be done in Matlab? */
				/* If not defined, then it should be set to NULL */
				ndim = mxGetNumberOfDimensions(tmp);
				dims = mxGetDimensions(tmp);
				miscell->parMap[jstruct].Range = mxGetPr(tmp);
			}

			tmp = mxGetField(prhs[1], jstruct, "Normalized");
			if (tmp != NULL)
			{
				miscell->parMap[jstruct].Normalized = mxGetScalar(tmp)>0.5;
				if (miscell->parMap[jstruct].Normalized)
				{
					tmp = mxGetField(prhs[1], jstruct, "NormBounds");
					if (tmp == NULL)
					{
						mexPrintf("%s%s \n", "Predicate: ", miscell->parMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: 'NormBounds' is empty/undefined when 'Normalized' is set to true!");
					}
					ndim = mxGetNumberOfDimensions(tmp);
					dims = mxGetDimensions(tmp);
					if (!(ndim == 2 && ((dims[0] == 1 && dims[1] == 1) || (dims[0] == 1 && dims[1] == 2) || (dims[0] == 2 && dims[1] == 1))))
					{
						mexPrintf("%s%s \n", "Predicate: ", miscell->parMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: 'NormBounds' should be a 1D or 2D vector!");
					}
					if (nrhs>4 && !((dims[0] == 1 && dims[1] == 2) || (dims[0] == 2 && dims[1] == 1)))
					{
						mexPrintf("%s%s \n", "Predicate: ", miscell->parMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: 'NormBounds' should be a 2D vector!");
					}
					miscell->parMap[jstruct].NormBounds = mxGetPr(tmp);
				}
				else
				{
					miscell->parMap[jstruct].Normalized = false;
					miscell->parMap[jstruct].NormBounds = NULL;
				}
			}
		}
		else
		{
			/* Get name of the predicate */
			ndim = mxGetNumberOfDimensions(tmp);
			dims = mxGetDimensions(tmp);
			buflen = dims[1]*sizeof(mxChar)+1;
			predMap[jstruct].str = (char *)emalloc(buflen); 
			miscell->predMap[jstruct].str = (char *)emalloc(buflen); 
			predMap[jstruct].set.idx = (int) jstruct;   
			predMap[jstruct].true_pred = true;
			status = mxGetString(tmp, predMap[jstruct].str, buflen);
			status = mxGetString(tmp, miscell->predMap[jstruct].str, buflen);   

			/* Get range*/
			tmp = mxGetField(prhs[1], jstruct, "range");
			if(tmp != NULL)
			{
				/* TODO: This should be moved when reading field b in order to check if dimensions match */
				/* Error checks that the bounds are correct? Should this be done in Matlab? */
				/* If not defined, then it should be set to NULL */
				ndim = mxGetNumberOfDimensions(tmp);
				dims = mxGetDimensions(tmp);
				predMap[jstruct].Range = mxGetPr(tmp);
				miscell->predMap[jstruct].Range = mxGetPr(tmp);
			}

			tmp = mxGetField(prhs[1], jstruct, "Normalized");
			if (tmp != NULL)
			{
				miscell->predMap[jstruct].Normalized = mxGetScalar(tmp)>0.5;
				predMap[jstruct].Normalized = miscell->predMap[jstruct].Normalized;
				if (miscell->predMap[jstruct].Normalized)
				{
					tmp = mxGetField(prhs[1], jstruct, "NormBounds");
					if (tmp == NULL)
					{
						mexPrintf("%s%s \n", "Predicate: ", miscell->predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: 'NormBounds' is empty/undefined when 'Normalized' is set to true!");
					}
					ndim = mxGetNumberOfDimensions(tmp);
					dims = mxGetDimensions(tmp);
					if (!(ndim == 2 && ((dims[0] == 1 && dims[1] == 1) || (dims[0] == 1 && dims[1] == 2) || (dims[0] == 2 && dims[1] == 1))))
					{
						mexPrintf("%s%s \n", "Predicate: ", miscell->predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: 'NormBounds' should be a 1D or 2D vector!");
					}
					if (nrhs>4 && !((dims[0] == 1 && dims[1] == 2) || (dims[0] == 2 && dims[1] == 1)))
					{
						mexPrintf("%s%s \n", "Predicate: ", miscell->predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: 'NormBounds' should be a 2D vector!");
					}
					predMap[jstruct].NormBounds = mxGetPr(tmp);
					miscell->predMap[jstruct].NormBounds = mxGetPr(tmp);
				}
				else
				{
					miscell->predMap[jstruct].Normalized = false;
					predMap[jstruct].Normalized = false;
					miscell->predMap[jstruct].NormBounds = NULL;
					predMap[jstruct].NormBounds = NULL;
				}

			}
			/* Get projection */         
			tmp = mxGetField(prhs[1], jstruct, "proj");
			if(tmp == NULL)
			{ 
                predMap[jstruct].set.nproj=0;
			}
            else
            {
                ndimP = mxGetNumberOfDimensions(tmp);
				if (ndimP>2)
				{
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The projection map is not in proper form!"); 
				}
				dimsP = mxGetDimensions(tmp);
                if(dimsP[0]!=1)
				{
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The projection map must be a row vector !"); 
				}
                if(dimsP[1]>miscell->dp_taliro_param.SysDim)
				{
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The projection map must have size less than or equal to the system dimentions !"); 
				}
				predMap[jstruct].set.nproj=dimsP[1];
				predMap[jstruct].set.proj=(int *)emalloc(predMap[jstruct].set.nproj*sizeof(int));
				tempI=0;
				for (iii=0; iii<predMap[jstruct].set.nproj; iii++)
				{
					predMap[jstruct].set.proj[iii] = (mxGetPr(tmp))[iii];
					if(predMap[jstruct].set.proj[iii]>miscell->dp_taliro_param.SysDim)
					{
						mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: The projection index must be less than or equal to the system dimentions !"); 
					}
					if(predMap[jstruct].set.proj[iii]<=tempI)
					{
						mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: The projection index must be incremental !"); 
					}
					else
					{
						tempI=predMap[jstruct].set.proj[iii];
					}
			    }
            }

			/* Get set */
			tmp = mxGetField(prhs[1], jstruct, "A");
			/* If A is empty, then we should have an interval */
			if(tmp == NULL)
			{ 
				tmp = mxGetField(prhs[1], jstruct, "Int");
				if(tmp == NULL) {
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: Both fields 'A' and 'Int' do not exist!"); 
				}
			}
			else if(mxIsEmpty(tmp))
			{ 
				predMap[jstruct].set.isSetRn = true;
				predMap[jstruct].set.ncon = 0;
			}
			else
			{
				predMap[jstruct].set.isSetRn = false;
				/* get A */
				if (mxIsCell(tmp)){
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: Field 'A' is a cell array.\nCell array is only supported for CLG guard map!");
				}
				ndimA = mxGetNumberOfDimensions(tmp);
				if (ndimA>2)
				{
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The set constraints are not in proper form!"); 
				}
				dimsA = mxGetDimensions(tmp);
                /* for projection is updated */
				if ((miscell->dp_taliro_param.SysDim != dimsA[1]&&predMap[jstruct].set.nproj==0)||(predMap[jstruct].set.nproj!=dimsA[1]&&predMap[jstruct].set.nproj>0))
				{
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The dimensions of the set constraints and the state trace do not match!"); 
				}
				predMap[jstruct].set.ncon = dimsA[0]; /* the number of constraints */
				if (predMap[jstruct].set.ncon>2 && miscell->dp_taliro_param.SysDim==1)
				{
					mexPrintf("%s%s \n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: For 1D signals only up to two constraints per predicate are allowed!\n More than two are redundant!"); 
				}
                /* for projection is added */
				predMap[jstruct].set.A = (double **)emalloc(predMap[jstruct].set.ncon*sizeof(double*));
				if(predMap[jstruct].set.nproj==0)
				{
					for (iii=0; iii<predMap[jstruct].set.ncon; iii++)
					{
						predMap[jstruct].set.A[iii] = (double *)emalloc(miscell->dp_taliro_param.SysDim*sizeof(double));
						for (jjj=0; jjj<miscell->dp_taliro_param.SysDim; jjj++)
							predMap[jstruct].set.A[iii][jjj] = (mxGetPr(tmp))[iii+jjj*predMap[jstruct].set.ncon];
					}
			    }
				else
				{
					for (iii=0; iii<predMap[jstruct].set.ncon; iii++)
					{
						predMap[jstruct].set.A[iii] = (double *)emalloc(predMap[jstruct].set.nproj*sizeof(double));
						for (jjj=0; jjj<predMap[jstruct].set.nproj; jjj++)
							predMap[jstruct].set.A[iii][jjj] = (mxGetPr(tmp))[iii+jjj*predMap[jstruct].set.ncon];
					}
				}
                /* for projection */

			
				/* get b */
				tmp = mxGetField(prhs[1], jstruct, "b");
				if(tmp == NULL) 
				{ 
					mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: Field 'b' is empty!"); 
				}
				if (mxIsCell(tmp)){
					mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: Field 'b' is a cell array.\nCell array is only supported for CLG guard map!");
				}
				ndimb = mxGetNumberOfDimensions(tmp);
				if (ndimb>2)
				{
					mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The set constraints are not in proper form!"); 
				}
				dimsb = mxGetDimensions(tmp);
				if (predMap[jstruct].set.ncon != dimsb[0])
				{
					mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
					mexErrMsgTxt("mx_dp_taliro: Above predicate: The number of constraints between A and b do not match!"); 
				}
				predMap[jstruct].set.b = mxGetPr(tmp);
				if (predMap[jstruct].set.ncon==2 && (miscell->dp_taliro_param.SysDim==1||predMap[jstruct].set.nproj==1))
				{
					if ((predMap[jstruct].set.A[0][0]>0 && predMap[jstruct].set.A[1][0]>0) || 
						(predMap[jstruct].set.A[0][0]<0 && predMap[jstruct].set.A[1][0]<0))
					{
						mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: The set has redundant constraints! Please remove redundant constraints."); 
					}
					if (!((predMap[jstruct].set.A[0][0]<0 && (predMap[jstruct].set.b[0]/predMap[jstruct].set.A[0][0]<=predMap[jstruct].set.b[1]/predMap[jstruct].set.A[1][0])) || 
						  (predMap[jstruct].set.A[1][0]<0 && (predMap[jstruct].set.b[1]/predMap[jstruct].set.A[1][0]<=predMap[jstruct].set.b[0]/predMap[jstruct].set.A[0][0]))))
					{
						mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
						mexErrMsgTxt("mx_dp_taliro: Above predicate: The set is empty! Please modify the constraints."); 
					}
				}
			}			
			/* get control locations */
			if (nrhs>4)
			{
				tmp = mxGetField(prhs[1], jstruct, "loc");
                if(is_Multi_HA==1){
                    predMap[jstruct].set.nloc=0;
					predMap[jstruct].set.locNCLG=(double **)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(double *));
					predMap[jstruct].set.nlocNCLG=(int *)emalloc((miscell->dp_taliro_param.nCLG)*sizeof(int));
					if(tmp == NULL){
	                    for(objs=0;objs<miscell->dp_taliro_param.nCLG;objs++){
							predMap[jstruct].set.nlocNCLG[objs]=0;
							predMap[jstruct].set.locNCLG[objs]=NULL;
						}
					}
					else
                    for(objs=0;objs<miscell->dp_taliro_param.nCLG;objs++){
                        tmp1 = mxGetCell(tmp,objs);
						if(tmp1 == NULL) 
    					{ 
							predMap[jstruct].set.nlocNCLG[objs]=0;
							predMap[jstruct].set.locNCLG[objs]=NULL;
                		}
						else
						{
		    				NCells = mxGetNumberOfElements(tmp1);
							ndim = mxGetNumberOfDimensions(tmp1);
							if (ndim>2)
							{
								mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
								mexErrMsgTxt("mx_dp_taliro: Above predicate: The control location vector is not in proper form!"); 
						    }
							dims = mxGetDimensions(tmp1);
			                if (dims[0]>1)
				            {
    							mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
        						mexErrMsgTxt("mx_dp_taliro: Above predicate: The control location vector must be row vector!"); 
            				}
                			predMap[jstruct].set.nlocNCLG[objs] = dims[1];
			                predMap[jstruct].set.locNCLG[objs] = mxGetPr(tmp1);
						}
			        }
			    }
                else{
    				NCells = mxGetNumberOfElements(tmp);
    				if(tmp == NULL) 
    				{ 
        				mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
            			mexErrMsgTxt("mx_dp_taliro: Above predicate: Field 'loc' is empty!"); 
                	}
                    ndim = mxGetNumberOfDimensions(tmp);
                    if (ndim>2)
                    {
                        mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
                        mexErrMsgTxt("mx_dp_taliro: Above predicate: The control location vector is not in proper form!"); 
                    }
                    dims = mxGetDimensions(tmp);
                    if (dims[0]>1)
                    {
    					mexPrintf("%s%s\n", "Predicate: ", predMap[jstruct].str);
        				mexErrMsgTxt("mx_dp_taliro: Above predicate: The control location vector must be row vector!"); 
            		}
                	predMap[jstruct].set.nloc = dims[1];
                    predMap[jstruct].set.loc = mxGetPr(tmp);
                }
			}
		}
    }

	miscell->tl_out = stdout; 

	/* set up some variables wrt to timing constraints */
	miscell->zero2inf.l_closed = 1;
	miscell->zero2inf.u_closed = 0;
	miscell->emptyInter.l_closed = 0;
	miscell->emptyInter.u_closed = 0;
	if (miscell->dp_taliro_param.ConOnSamples)
	{	
		miscell->zero.numi.inf = 0;
		miscell->zero.numi.i_num = 0;
		miscell->inf.numi.inf = 1;
		miscell->inf.numi.i_num = 0;
		miscell->zero2inf.lbd = miscell->zero; 
		miscell->zero2inf.ubd = miscell->inf;
		miscell->emptyInter.lbd = miscell->zero;
		miscell->emptyInter.ubd = miscell->zero;
	}
	else
	{	
		miscell->zero.numf.inf = 0;
		miscell->zero.numf.f_num = 0.0;
		miscell->inf.numf.inf = 1;
		miscell->inf.numf.f_num = 0.0;
		miscell->zero2inf.lbd = miscell->zero;
		miscell->zero2inf.ubd = miscell->inf;
		miscell->emptyInter.lbd = miscell->zero;
		miscell->emptyInter.ubd = miscell->zero;
	}

	node = tl_parse(cnt,hasuform,uform, miscell, tl_yychar);

	if (miscell->dp_taliro_param.LTL==0 && nrhs<4)
		mexErrMsgTxt("mx_dp_taliro: The formula is in MTL, but there are no timestamps!"); 

	plhs[0] = DynamicProgramming(node,predMap,XTrace,TStamps,LTrace,&distData,&(miscell->dp_taliro_param), miscell);

	for (iii=0; iii<miscell->dp_taliro_param.true_nPred; iii++)
	{
		tl_clearlookup(predMap[iii].str, miscell);
		for (jjj=0;jjj<predMap[iii].set.ncon;jjj++)
			mxFree(predMap[iii].set.A[jjj]);
        mxFree(predMap[iii].str);
		mxFree(predMap[iii].set.A);
	}
	for (iii=0; iii<miscell->dp_taliro_param.true_nPred; iii++)
	{
        mxFree(miscell->predMap[iii].str);
        mxFree(miscell->parMap[iii].str);
    }    
    mxFree(miscell->pList.pindex);
    mxFree(miscell->predMap);
    mxFree(miscell->parMap);
    mxFree(predMap);
    /* for projection and multiple H.A.s is updated */
    if (nrhs>7)
	{
        if(is_Multi_HA==0){
			for (iii=0; iii<miscell->dp_taliro_param.tnLoc; iii++)
			{
				for (jjj=0; jjj<distData.AdjLNell[iii]; jjj++)
				{
    				for (kk=0; kk<distData.GuardMap[iii][jjj].nset; kk++)
					{
	                    for (i1=0; i1<distData.GuardMap[iii][jjj].ncon[kk]; i1++){
		                    mxFree(distData.GuardMap[iii][jjj].A[kk][i1]);
			            }
				        mxFree(distData.GuardMap[iii][jjj].A[kk]);
					}
					mxFree(distData.GuardMap[iii][jjj].A);
					mxFree(distData.GuardMap[iii][jjj].b);
					mxFree(distData.GuardMap[iii][jjj].ncon);				
				}
				mxFree(distData.GuardMap[iii]);
			}
			mxFree(distData.GuardMap);
			mxFree(distData.AdjLNell);
			mxFree(distData.AdjL);
		}
		else{
           for(objs=0;objs<miscell->dp_taliro_param.nCLG;objs++){
               for (iii=0; iii<miscell->dp_taliro_param.tnLocNCLG[objs]; iii++){
                   for (jjj=0; jjj<distData.AdjLNellNCLG[objs][iii]; jjj++){
                       for (kk=0; kk<distData.GuardMapNCLG[objs][iii][jjj].nset; kk++){
    	                    for (i1=0; i1<distData.GuardMapNCLG[objs][iii][jjj].ncon[kk]; i1++){
        	                    mxFree(distData.GuardMapNCLG[objs][iii][jjj].A[kk][i1]);
            	            }
                	        mxFree(distData.GuardMapNCLG[objs][iii][jjj].A[kk]);
                       }
                       mxFree(distData.GuardMapNCLG[objs][iii][jjj].A);
                       mxFree(distData.GuardMapNCLG[objs][iii][jjj].b);
                       mxFree(distData.GuardMapNCLG[objs][iii][jjj].ncon);				
                   }
                   mxFree(distData.GuardMapNCLG[objs][iii]);
               }
               mxFree(distData.GuardMapNCLG[objs]);
               mxFree(distData.AdjLNellNCLG[objs]);
               mxFree(distData.AdjLnCLG[objs]);
           }
           mxFree(distData.GuardMapNCLG);
           mxFree(distData.AdjLNellNCLG);
           mxFree(distData.AdjLnCLG);
		}
	}
    
    mxFree(miscell);
	mxFree(tl_yychar);
}


