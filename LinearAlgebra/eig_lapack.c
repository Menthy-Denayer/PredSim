#include "mex.h"
#include <lapacke.h>

/* 
 * Usage in MATLAB:
 *   [V,D] = eig_lapack(C)
 *   C must be symmetric
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if(nrhs != 1) mexErrMsgTxt("One input required.");
    if(nlhs != 2) mexErrMsgTxt("Two outputs required: [V,D]");

    mwSize n = mxGetN(prhs[0]);
    double *C = mxGetPr(prhs[0]);

    // Copy C for LAPACKE (it overwrites input)
    plhs[0] = mxCreateDoubleMatrix(n,n,mxREAL);
    double *V = mxGetPr(plhs[0]);
    for(mwSize i=0; i<n*n; i++) V[i] = C[i];

    // Allocate eigenvalues vector
    double *eigvals = (double*) mxCalloc(n, sizeof(double));

    // Compute eigenvalues and eigenvectors
    lapack_int info = LAPACKE_dsyev(LAPACK_COL_MAJOR, 'V', 'U', n, V, n, eigvals);
    if(info != 0) {
        mxFree(eigvals);
        mexErrMsgTxt("Eigen decomposition failed.");
    }

    // Convert eigenvalues to diagonal matrix D
    plhs[1] = mxCreateDoubleMatrix(n,n,mxREAL);
    double *D = mxGetPr(plhs[1]);
    for(mwSize i=0; i<n; i++) {
        D[i*n + i] = eigvals[i];   // column-major
    }

    mxFree(eigvals);
}
