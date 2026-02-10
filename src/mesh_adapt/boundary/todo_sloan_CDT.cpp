
//~ - no es óptimo
//~ - no hace flips de mejora
//~ - no garantiza Delaunay perfecto (pero sí CDT)

//~ Para remallado: es más que suficiente.

inline double orient2d(const Vec2& a, const Vec2& b, const Vec2& c)
{
    return (b.x - a.x)*(c.y - a.y)
         - (b.y - a.y)*(c.x - a.x);
}

bool segments_intersect(
    const Vec2& a, const Vec2& b,
    const Vec2& c, const Vec2& d
){
    double o1 = orient2d(a,b,c);
    double o2 = orient2d(a,b,d);
    double o3 = orient2d(c,d,a);
    double o4 = orient2d(c,d,b);

    return (o1*o2 < 0) && (o3*o4 < 0);
}



bool edge_exists(int a, int b) const
{
    for(const auto& t : triangles){
        int cnt = 0;
        if(t[0]==a || t[1]==a || t[2]==a) cnt++;
        if(t[0]==b || t[1]==b || t[2]==b) cnt++;
        if(cnt==2) return true;
    }
    return false;
}

std::vector<int> find_intersected_triangles(int a, int b)
{
    std::vector<int> bad;

    const Vec2& A = points[a];
    const Vec2& B = points[b];

    for(int i=0;i<(int)triangles.size();++i){
        const auto& t = triangles[i];

        for(int e=0;e<3;++e){
            int i0 = t[e];
            int i1 = t[(e+1)%3];

            // ignorar si comparten vértice
            if(i0==a || i0==b || i1==a || i1==b) continue;

            if(segments_intersect(A,B, points[i0], points[i1])){
                bad.push_back(i);
                break;
            }
        }
    }
    return bad;
}


std::vector<Edge2D> cavity_boundary(
    const std::vector<int>& bad_ids
){
    std::map<std::pair<int,int>, int> count;

    auto add = [&](int a,int b){
        if(a>b) std::swap(a,b);
        count[{a,b}]++;
    };

    for(int id : bad_ids){
        const auto& t = triangles[id];
        add(t[0],t[1]);
        add(t[1],t[2]);
        add(t[2],t[0]);
    }

    std::vector<Edge2D> poly;
    for(auto& [e,c] : count){
        if(c==1)
            poly.emplace_back(e.first, e.second);
    }
    return poly;
}


////(sí, esta parte se puede refinar; esta versión es deliberadamente clara, no óptima)
void retriangulate_with_constraint(
    const std::vector<Edge2D>& poly,
    int a, int b
){
    // insertar la constraint
    triangles.push_back({a,b,poly[0].a}); // semilla

    for(const auto& e : poly){
        if(e.a==a || e.a==b || e.b==a || e.b==b)
            continue;

        int x = e.a;
        int y = e.b;

        if(orient2d(points[a],points[b],points[x])>0)
            triangles.push_back({a,x,b});
        else
            triangles.push_back({a,b,x});

        if(orient2d(points[a],points[b],points[y])>0)
            triangles.push_back({a,y,b});
        else
            triangles.push_back({a,b,y});
    }
}


void enforce_constraint(int a, int b)
{
    if(edge_exists(a,b)) return;

    auto bad = find_intersected_triangles(a,b);
    if(bad.empty()) return;

    auto poly = cavity_boundary(bad);
    erase_triangles(bad);
    retriangulate_with_constraint(poly,a,b);
}

void enforce_all_constraints()
{
    for(const auto& e : constraints){
        enforce_constraint(e.a, e.b);
    }
}

