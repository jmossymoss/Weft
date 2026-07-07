// Decoupled-face spike (decoupled-core de-risk, step 2).
//
// Proves the per-face model end to end, self-contained (no OCCT): a face
// meshes its INTERIOR as a uniform grid at its OWN (nu x nv) count, and each of
// its four borders is sampled at a DIFFERENT neighbour-agreed count. Each side
// is absorbed with an open loop-bridge (quads + grouped n-gons), corners shared
// so adjacent sides meet exactly. Result must be watertight (every interior
// edge shared by 2 cells; only the outer boundary open), manifold, fold-free.
//
// If this holds, a decoupled mesher is: grid the interior at your own count,
// sample each shared edge once at the neighbour count, bridge the gap. No
// global density solve, no ripple, full per-face control.
//   g++ -O2 -std=c++17 core/spike/decoupled_face.cpp -o /tmp/df && /tmp/df
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

using V3 = std::array<double, 3>;
struct Mesh { std::vector<V3> verts; std::vector<std::vector<int>> polys; };

static int addV(Mesh& m, double x, double y) {
    m.verts.push_back({x, y, 0}); return (int)m.verts.size() - 1;
}

// Open bridge: absorb polyline A (a verts, indices) against polyline B (b
// verts), sharing endpoints A[0]==B[0]-corner and A.back()==B.back()-corner in
// SPACE (they are distinct vertices at the same-ish position — here A is the
// interior row, B the outer edge, offset inward). One cell per coarse edge.
static void bridgeStrip(const std::vector<int>& A, const std::vector<int>& B,
                        Mesh& out) {
    const int a = (int)A.size() - 1, b = (int)B.size() - 1;  // edge counts
    if (a < 1 || b < 1) return;
    auto emit = [&](std::vector<int> c) {
        std::vector<int> cl;
        for (int i : c) if (cl.empty() || cl.back() != i) cl.push_back(i);
        if (cl.size() > 1 && cl.front() == cl.back()) cl.pop_back();
        if ((int)cl.size() >= 3) out.polys.push_back(std::move(cl));
    };
    if (b <= a) {  // outer coarse: one cell per outer edge, absorb inner run
        for (int c = 0; c < b; ++c) {
            int ia = (int)std::llround((long double)c * a / b);
            int ib = (int)std::llround((long double)(c + 1) * a / b);
            std::vector<int> cell;
            cell.push_back(B[c]); cell.push_back(B[c + 1]);
            for (int k = ib; k >= ia; --k) cell.push_back(A[k]);
            emit(std::move(cell));
        }
    } else {  // inner coarse
        for (int c = 0; c < a; ++c) {
            int ia = (int)std::llround((long double)c * b / a);
            int ib = (int)std::llround((long double)(c + 1) * b / a);
            std::vector<int> cell;
            for (int k = ia; k <= ib; ++k) cell.push_back(B[k]);
            cell.push_back(A[c + 1]); cell.push_back(A[c]);
            emit(std::move(cell));
        }
    }
}

struct Rep { int cells=0,quad=0,ngon=0,tri=0,openInt=0,nonMf=0,fold=0,degen=0; };

static Rep validate(const Mesh& m, const std::map<std::pair<int,int>,int>& bnd){
    Rep r; std::map<std::pair<int,int>,int> use;
    for (auto& p : m.polys) {
        r.cells++;
        if (p.size()==3) r.tri++; else if (p.size()==4) r.quad++; else r.ngon++;
        double area=0;  // signed area in XY (all cells CCW => +)
        for (size_t k=0;k<p.size();++k){ auto&A=m.verts[p[k]];auto&B=m.verts[p[(k+1)%p.size()]];
            area += A[0]*B[1]-B[0]*A[1]; }
        if (std::abs(area)<1e-9) r.degen++; else if (area<0) r.fold++;
        for (size_t k=0;k<p.size();++k){int A=p[k],B=p[(k+1)%p.size()];
            use[{std::min(A,B),std::max(A,B)}]++;}
    }
    for (auto&[e,n]:use){ if(n>2)r.nonMf++; else if(n==1 && !bnd.count(e)) r.openInt++; }
    return r;
}

// Build a decoupled square face: interior grid nu x nv inset by `ins`, four
// outer edges at counts Nt,Nr,Nb,Nl, bridged per side with shared corners.
static Rep buildFace(int nu,int nv,int Nt,int Nr,int Nb,int Nl){
    Mesh m; const double ins=0.12;
    // interior grid verts (nv+1) rows x (nu+1) cols in [ins,1-ins]
    std::vector<std::vector<int>> g(nv+1, std::vector<int>(nu+1));
    for(int r=0;r<=nv;++r)for(int c=0;c<=nu;++c)
        g[r][c]=addV(m, ins+(1-2*ins)*c/nu, ins+(1-2*ins)*r/nv);
    for(int r=0;r<nv;++r)for(int c=0;c<nu;++c)
        m.polys.push_back({g[r][c],g[r][c+1],g[r+1][c+1],g[r+1][c]});
    // outer boundary verts per edge (corners at unit square). Order CCW:
    // bottom L->R, right B->T, top R->L, left T->B. Corners shared.
    auto edge=[&](double x0,double y0,double x1,double y1,int n,bool inclFirst){
        std::vector<int> e; for(int k=(inclFirst?0:1);k<=n;++k)
            e.push_back(addV(m,x0+(x1-x0)*k/n,y0+(y1-y0)*k/n)); return e; };
    int c00=addV(m,0,0),c10=addV(m,1,0),c11=addV(m,1,1),c01=addV(m,0,1);
    std::vector<int> ob; ob.push_back(c00);
    for(int i:edge(0,0,1,0,Nb,false)) ob.push_back(i); ob.back()=c10;
    for(int i:edge(1,0,1,1,Nr,false)) ob.push_back(i); ob.back()=c11;
    for(int i:edge(1,1,0,1,Nt,false)) ob.push_back(i); ob.back()=c01;
    for(int i:edge(0,1,0,0,Nl,false)) ob.push_back(i); ob.pop_back();
    // interior boundary rows/cols with matching corners
    std::vector<int> ibBot,ibRight,ibTop,ibLeft;
    for(int c=0;c<=nu;++c) ibBot.push_back(g[0][c]);
    for(int r=0;r<=nv;++r) ibRight.push_back(g[r][nu]);
    for(int c=nu;c>=0;--c) ibTop.push_back(g[nv][c]);
    for(int r=nv;r>=0;--r) ibLeft.push_back(g[r][0]);
    // outer edge vert lists (with corners)
    auto slice=[&](int start,int cnt){ std::vector<int> s;
        for(int k=0;k<=cnt;++k) s.push_back(ob[(start+k)%ob.size()]); return s; };
    int p=0;
    std::vector<int> oBot=slice(p,Nb); p+=Nb;
    std::vector<int> oRight=slice(p,Nr); p+=Nr;
    std::vector<int> oTop=slice(p,Nt); p+=Nt;
    std::vector<int> oLeft=slice(p,Nl);
    bridgeStrip(ibBot,oBot,m);
    bridgeStrip(ibRight,oRight,m);
    bridgeStrip(ibTop,oTop,m);
    bridgeStrip(ibLeft,oLeft,m);
    // boundary edge set = the outer loop
    std::map<std::pair<int,int>,int> bnd;
    for(size_t k=0;k<ob.size();++k){int A=ob[k],B=ob[(k+1)%ob.size()];
        bnd[{std::min(A,B),std::max(A,B)}]++;}
    return validate(m,bnd);
}

int main(){
    struct C{int nu,nv,Nt,Nr,Nb,Nl;const char*name;};
    std::vector<C> cs={
        {8,8, 8,8,8,8, "8x8 matched"},
        {8,8, 16,4,8,12, "8x8 all diff"},
        {12,6, 3,20,12,5, "12x6 wild borders"},
        {16,16, 1,1,32,1, "16x16 one dense side"},
        {4,10, 24,7,4,31, "4x10 coprime"},
        {20,20, 40,3,40,3, "20x20 lopsided"},
        {6,6, 2,2,2,2, "6x6 coarse borders"},
    };
    printf("%-22s %6s %5s %5s %5s | %7s %5s %5s %5s\n","face","cells","quad","ngon","tri","openInt","nonMf","fold","degen");
    int pass=0;
    for(auto&c:cs){
        Rep r=buildFace(c.nu,c.nv,c.Nt,c.Nr,c.Nb,c.Nl);
        bool ok=r.openInt==0&&r.nonMf==0&&r.fold==0&&r.degen==0&&r.cells>0;
        pass+=ok;
        printf("%-22s %6d %5d %5d %5d | %7d %5d %5d %5d  %s\n",c.name,r.cells,r.quad,r.ngon,r.tri,r.openInt,r.nonMf,r.fold,r.degen, ok?"OK":"FAIL");
    }
    printf("\n%d/%d decoupled faces clean\n",pass,(int)cs.size());
    return pass==(int)cs.size()?0:1;
}
