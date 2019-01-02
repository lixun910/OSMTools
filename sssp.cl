
int maskArrayEmpty(__global int *maskArray, int vertexCount)
{
    for(size_t i = 0; i < vertexCount; i++ ) {
        if (maskArray[i] == 1) {
            return 0;
        }
    }
    return 1;
}

__kernel void SSSP_KERNEL(int vertexCount, int edgeCount,
                          __global int *vertexArray,
                          __global int *edgeArray,
                          __global int *weightArray,
                          __global int *maskArray,
                          __global int *costArray,
                          __global int *updatingCostArray,
                          __global int *sourceVertices,
                          __global int *results)
{
    // access thread id
    int tid = get_global_id(0);

    if (tid >= vertexCount) return;

    int i;
    // Initialize the buffer for this run
    for (i = 0; i < vertexCount; i++) {
        if (i == sourceVertices[tid]) {
            maskArray[i] = 1;
            costArray[i] = 0;
            updatingCostArray[i] = 0;
        } else {
            maskArray[i] = 0;
            costArray[i] = INT_MAX;
            updatingCostArray[i] = INT_MAX;
        }
    }

    int edgeStart;
    int edgeEnd;
    int edge;
    int nid;
    int val;
    while(maskArrayEmpty(maskArray, vertexCount) == 0) {
        for (i=0; i< vertexCount; ++i) {
            if (maskArray[i] != 0) {
                maskArray[i] = 0;
                edgeStart = vertexArray[i];
                if (i + 1 < vertexCount) {
                    edgeEnd = vertexArray[i + 1];
                } else {
                    edgeEnd = edgeCount;
                }
                for (edge=edgeStart; edge<edgeEnd; ++edge) {
                    nid = edgeArray[edge];
                    val = costArray[i] + weightArray[edge];
                    if (updatingCostArray[nid] > val) {
                        updatingCostArray[nid] = val;
                    }
                }
            }
        }
        for (i=0; i<vertexCount; ++i) {
            if (costArray[i] > updatingCostArray[i]) {
                costArray[i] = updatingCostArray[i];
                maskArray[i] = 1;
            }
            updatingCostArray[i] = costArray[i];
        }
    }
    // copy results
    size_t offset = tid * vertexCount;
    for (i=0; i<vertexCount; ++i) {
        results[offset+i] = costArray[i];
    }
}
