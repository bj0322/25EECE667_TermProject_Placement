#include "Circuit.h"
#include "matrixSolver.h"

#include <vector>
#include <unordered_map>
#include <valarray>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>
#include <iomanip>
#include <random> 
#include <chrono> 
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace Placer {
    template <typename T>
    inline T clamp(T val, T minVal, T maxVal) {
        if (maxVal < minVal) return minVal;
        return std::max(minVal, std::min(maxVal, val));
    }

    bool checkOverlap(double x1, double y1, double w1, double h1,
        double x2, double y2, double w2, double h2) {
        return x1 < x2 + w2 && x1 + w1 > x2 &&
            y1 < y2 + h2 && y1 + h1 > y2;
    }

    double computeNetHPWL(Net* net) {
        const auto& pins = net->getConnectedPins();

        double minX = 1e15, minY = 1e15;
        double maxX = -1e15, maxY = -1e15;
        bool connected = false;

        for (Pin* p : pins) {
            int cx, cy;
            if (p->isInstancePin()) {
                Instance* inst = p->getInstance();
                if (!inst) continue;
                auto coord = inst->getCoordinate();
                cx = coord.first;
                cy = coord.second;
            }
            else {
                auto coord = p->getCoordinate();
                cx = coord.first;
                cy = coord.second;
            }
            if (cx < minX) minX = cx;
            if (cx > maxX) maxX = cx;
            if (cy < minY) minY = cy;
            if (cy > maxY) maxY = cy;
            connected = true;
        }
        if (!connected) return 0.0;
        return (maxX - minX) + (maxY - minY);
    }

    void legalization(
        std::vector<Instance*>& instances,
        double coreLX, double coreLY, double coreW, double coreH,
        double coreUX, double coreUY
    ) {
        int n = (int)instances.size();
        if (n == 0) return;

        int rowHeight = instances[0]->getHeight();
        for (auto* inst : instances) {
            if (inst->getHeight() < rowHeight && inst->getHeight() > 0) rowHeight = inst->getHeight();
        }
        if (rowHeight <= 0) rowHeight = 2000;

        int numRows = (int)((coreUY - coreLY) / rowHeight);
        if (numRows <= 0) numRows = 1;

        struct CellInfo {
            int id;
            double x, y;
            double w;
        };
        std::vector<CellInfo> cells;
        cells.reserve(n);

        for (int i = 0; i < n; ++i) {
            if (instances[i]->isPlaced()) continue;

            auto coord = instances[i]->getCoordinate();
            double w = (double)instances[i]->getWidth();
            double h = (double)instances[i]->getHeight();
            double ix = clamp((double)coord.first, coreLX, coreUX - w);
            double iy = clamp((double)coord.second, coreLY, coreUY - h);
            cells.push_back({ i, ix, iy, w });
        }

        std::sort(cells.begin(), cells.end(), [&](const CellInfo& a, const CellInfo& b) {
            int rowA = clamp((int)((a.y - coreLY) / rowHeight), 0, numRows - 1);
            int rowB = clamp((int)((b.y - coreLY) / rowHeight), 0, numRows - 1);
            if (rowA != rowB) return rowA < rowB;
            return a.x < b.x;
            });

        std::vector<double> rowUsage(numRows, coreLX);

        for (const auto& cell : cells) {
            int bestRow = -1;
            double bestCost = std::numeric_limits<double>::max();
            double bestX = coreLX;

            int idealRow = clamp((int)((cell.y - coreLY) / rowHeight), 0, numRows - 1);
            int searchRange = std::max(5, numRows / 10);

            for (int r = std::max(0, idealRow - searchRange); r <= std::min(numRows - 1, idealRow + searchRange); ++r) {
                double placeX = std::max(rowUsage[r], cell.x);
                if (placeX + cell.w > coreUX) continue;

                double dispX = placeX - cell.x;
                double dispY = (coreLY + r * rowHeight) - cell.y;
                double cost = std::abs(dispX) + std::abs(dispY) * 5.0;

                if (cost < bestCost) {
                    bestCost = cost;
                    bestRow = r;
                    bestX = placeX;
                }
            }

            if (bestRow != -1) {
                Instance* inst = instances[cell.id];
                inst->setCoordinate((int)bestX, (int)(coreLY + bestRow * rowHeight));
                rowUsage[bestRow] = bestX + cell.w;
            }
            else {
                int fallbackRow = 0;
                double minUsage = std::numeric_limits<double>::max();
                for (int r = 0; r < numRows; ++r) {
                    if (rowUsage[r] < minUsage && rowUsage[r] + cell.w <= coreUX) {
                        minUsage = rowUsage[r];
                        fallbackRow = r;
                    }
                }
                if (minUsage == std::numeric_limits<double>::max()) {
                    fallbackRow = 0;
                    minUsage = rowUsage[0];
                }
                Instance* inst = instances[cell.id];
                double finalX = std::min(minUsage, coreUX - cell.w);
                finalX = std::max(finalX, coreLX);
                inst->setCoordinate((int)finalX, (int)(coreLY + fallbackRow * rowHeight));
                rowUsage[fallbackRow] = finalX + cell.w;
            }
        }
    }

    void Circuit::myPlacement() {
        const int n = static_cast<int>(instance_pointers_.size());
        if (n == 0) return;

        double coreLX = 1e15, coreLY = 1e15;
        double coreUX = -1e15, coreUY = -1e15;
        bool pinFound = false;

        for (Net* net : net_pointers_) {
            for (Pin* pin : net->getConnectedPins()) {
                if (pin->isBlockPin()) {
                    auto c = pin->getCoordinate();
                    if (c.first < coreLX) coreLX = c.first;
                    if (c.second < coreLY) coreLY = c.second;
                    if (c.first > coreUX) coreUX = c.first;
                    if (c.second > coreUY) coreUY = c.second;
                    pinFound = true;
                }
            }
        }

        double totalCellArea = 0.0;
        int stdHeight = 2000;
        if (n > 0) stdHeight = instance_pointers_[0]->getHeight();

        for (auto* inst : instance_pointers_) {
            totalCellArea += ((double)inst->getWidth() * (double)inst->getHeight());
            if (inst->getHeight() < stdHeight && inst->getHeight() > 0) stdHeight = inst->getHeight();
        }

        double coreW = coreUX - coreLX;
        double coreH = coreUY - coreLY;

        if (!pinFound || coreW < 100.0 || coreH < 100.0) {
            double estimatedSide = std::sqrt(totalCellArea / 0.85);
            if (estimatedSide < 5000.0) estimatedSide = 10000.0;
            coreLX = 0.0; coreLY = 0.0;
            coreUX = estimatedSide; coreUY = estimatedSide;
            coreW = estimatedSide; coreH = estimatedSide;
        }

        struct Rect { double x, y, w, h; };
        std::vector<Rect> obstacles;
        std::vector<int> movableIndices;
        std::unordered_map<Instance*, int> instToIndex;

        for (int i = 0; i < n; ++i) {
            Instance* inst = instance_pointers_[i];
            instToIndex[inst] = i;

            if (inst->isPlaced()) {
                auto c = inst->getCoordinate();
                obstacles.push_back({ (double)c.first, (double)c.second, (double)inst->getWidth(), (double)inst->getHeight() });
            }
            else {
                movableIndices.push_back(i);
            }
        }

        std::vector<int> sortedIndices;
        sortedIndices.reserve(movableIndices.size());
        std::vector<bool> visited(n, false);

        std::vector<std::vector<int>> adj(n);
        for (Net* net : net_pointers_) {
            const auto& pins = net->getConnectedPins();
            if (pins.size() > 50) continue;

            std::vector<int> netCells;
            for (Pin* p : pins) {
                if (p->isInstancePin()) {
                    auto it = instToIndex.find(p->getInstance());
                    if (it != instToIndex.end() && !instance_pointers_[it->second]->isPlaced()) {
                        netCells.push_back(it->second);
                    }
                }
            }
            for (size_t i = 0; i < netCells.size(); ++i) {
                for (size_t j = i + 1; j < netCells.size(); ++j) {
                    adj[netCells[i]].push_back(netCells[j]);
                    adj[netCells[j]].push_back(netCells[i]);
                }
            }
        }

        for (int startNode : movableIndices) {
            if (visited[startNode]) continue;
            std::queue<int> q;
            q.push(startNode);
            visited[startNode] = true;
            sortedIndices.push_back(startNode);

            while (!q.empty()) {
                int u = q.front();
                q.pop();
                for (int v : adj[u]) {
                    if (!visited[v]) {
                        visited[v] = true;
                        sortedIndices.push_back(v);
                        q.push(v);
                    }
                }
            }
        }

        double marginW = coreW * 0.10;
        double marginH = coreH * 0.10;
        double placeLX = coreLX + marginW;
        double placeLY = coreLY + marginH;
        double placeW = coreW - 2 * marginW;
        double placeH = coreH - 2 * marginH;

        int numMovable = sortedIndices.size();
        int cols = std::ceil(std::sqrt(numMovable * 1.5));
        int rows = std::ceil((double)numMovable * 1.5 / cols);
        double slotW = placeW / cols;
        double slotH = placeH / rows;

        struct Point { double x, y; };
        std::vector<Point> validSlots;
        validSlots.reserve(numMovable + 1000);

        for (int r = 0; r < rows; ++r) {
            if (r % 2 == 0) {
                for (int c = 0; c < cols; ++c) {
                    double cx = placeLX + c * slotW;
                    double cy = placeLY + r * slotH;
                    bool conflict = false;
                    for (const auto& obs : obstacles) {
                        if (checkOverlap(cx, cy, slotW, slotH, obs.x, obs.y, obs.w, obs.h)) {
                            conflict = true; break;
                        }
                    }
                    if (!conflict) validSlots.push_back({ cx, cy });
                }
            }
            else {
                for (int c = cols - 1; c >= 0; --c) {
                    double cx = placeLX + c * slotW;
                    double cy = placeLY + r * slotH;
                    bool conflict = false;
                    for (const auto& obs : obstacles) {
                        if (checkOverlap(cx, cy, slotW, slotH, obs.x, obs.y, obs.w, obs.h)) {
                            conflict = true; break;
                        }
                    }
                    if (!conflict) validSlots.push_back({ cx, cy });
                }
            }
        }

        std::mt19937 gen(42);
        std::uniform_real_distribution<> randX(placeLX, placeLX + placeW);
        std::uniform_real_distribution<> randY(placeLY, placeLY + placeH);

        for (int i = 0; i < numMovable; ++i) {
            int cellIdx = sortedIndices[i];
            double finalX, finalY;

            if (i < validSlots.size()) {
                finalX = validSlots[i].x;
                finalY = validSlots[i].y;
            }
            else {
                finalX = randX(gen);
                finalY = randY(gen);
            }
            finalX = clamp(finalX, coreLX, coreUX - instance_pointers_[cellIdx]->getWidth());
            finalY = clamp(finalY, coreLY, coreUY - instance_pointers_[cellIdx]->getHeight());
            instance_pointers_[cellIdx]->setCoordinate((int)finalX, (int)finalY);
        }

        std::unordered_map<Instance*, std::vector<Net*>> instNetMap;
        instNetMap.reserve(n);
        for (Net* net : net_pointers_) {
            for (Pin* pin : net->getConnectedPins()) {
                if (pin->isInstancePin()) {
                    instNetMap[pin->getInstance()].push_back(net);
                }
            }
        }

        double T = 30000.0;
        double alpha = 0.8;

        int movesPerTemp = std::min((int)(numMovable / 20), 400);
        int maxSteps = 10;

        std::uniform_int_distribution<> distIdx(0, numMovable - 1);
        std::uniform_real_distribution<> distProb(0.0, 1.0);

        std::vector<Net*> affectedNets;
        affectedNets.reserve(100);

        for (int step = 0; step < maxSteps; ++step) {
            if (step % 2 == 0) std::cout << "Step " << step << "/" << maxSteps << std::endl;
            if (T < 0.1) break;

            for (int k = 0; k < movesPerTemp; ++k) {
                int rA = distIdx(gen);
                int rB = distIdx(gen);
                if (rA == rB) continue;

                int idxA = sortedIndices[rA];
                int idxB = sortedIndices[rB];

                Instance* instA = instance_pointers_[idxA];
                Instance* instB = instance_pointers_[idxB];

                affectedNets.clear();
                if (instNetMap.count(instA)) affectedNets.insert(affectedNets.end(), instNetMap[instA].begin(), instNetMap[instA].end());
                if (instNetMap.count(instB)) affectedNets.insert(affectedNets.end(), instNetMap[instB].begin(), instNetMap[instB].end());

                if (affectedNets.empty()) continue;

                std::sort(affectedNets.begin(), affectedNets.end());
                affectedNets.erase(std::unique(affectedNets.begin(), affectedNets.end()), affectedNets.end());

                double costBefore = 0.0;
                for (Net* net : affectedNets) costBefore += computeNetHPWL(net);

                auto coordA = instA->getCoordinate();
                auto coordB = instB->getCoordinate();

                double bx = clamp((double)coordA.first, coreLX, coreUX - instB->getWidth());
                double by = clamp((double)coordA.second, coreLY, coreUY - instB->getHeight());
                double ax = clamp((double)coordB.first, coreLX, coreUX - instA->getWidth());
                double ay = clamp((double)coordB.second, coreLY, coreUY - instA->getHeight());

                instA->setCoordinate((int)ax, (int)ay);
                instB->setCoordinate((int)bx, (int)by);

                double costAfter = 0.0;
                for (Net* net : affectedNets) costAfter += computeNetHPWL(net);

                double delta = costAfter - costBefore;

                if (delta < 0) {
                }
                else {
                    double prob = std::exp(-delta / T);
                    if (distProb(gen) < prob) {
                    }
                    else {
                        instA->setCoordinate(coordA.first, coordA.second);
                        instB->setCoordinate(coordB.first, coordB.second);
                    }
                }
            }
            T *= alpha;
        }

        legalization(instance_pointers_, coreLX, coreLY, coreW, coreH, coreUX, coreUY);
    }

    void Circuit::quadraticPlacement() {
        const int n = static_cast<int>(instance_pointers_.size());
        if (n == 0) return;

        std::unordered_map<Instance*, int> inst_index;
        inst_index.reserve(n);
        for (int i = 0; i < n; ++i) inst_index[instance_pointers_[i]] = i;

        struct Entry { int r; int c; double v; };
        std::vector<Entry> entries;
        entries.reserve(n * 100);

        std::valarray<double> bx(0.0, n);
        std::valarray<double> by(0.0, n);

        std::vector<int> movable_ids;
        std::vector<std::pair<double, double>> fixed_coords;

        for (Net* net : net_pointers_) {
            movable_ids.clear();
            fixed_coords.clear();
            const std::vector<Pin*> pins = net->getConnectedPins();
            for (Pin* pin : pins) {
                if (pin->isInstancePin()) {
                    Instance* inst = pin->getInstance();
                    if (!inst) continue;
                    auto it = inst_index.find(inst);
                    if (it != inst_index.end()) movable_ids.push_back(it->second);
                }
                else if (pin->isBlockPin()) {
                    std::pair<int, int> coord = pin->getCoordinate();
                    fixed_coords.emplace_back((double)coord.first, (double)coord.second);
                }
            }
            const int k = static_cast<int>(movable_ids.size() + fixed_coords.size());
            if (k < 2) continue;
            double w = (1.0 / (double)(k - 1)) * std::max(1, net->getWeight());
            const size_t m = movable_ids.size();
            if (m <= 2000) {
                for (size_t a = 0; a < m; ++a) {
                    for (size_t b = a + 1; b < m; ++b) {
                        int i = movable_ids[a]; int j = movable_ids[b];
                        entries.push_back({ i,i,w }); entries.push_back({ j,j,w });
                        entries.push_back({ i,j,-w }); entries.push_back({ j,i,-w });
                    }
                }
            }
            else {
                int ref = movable_ids[0];
                for (size_t idx = 1; idx < m; ++idx) {
                    int j = movable_ids[idx];
                    entries.push_back({ ref,ref,w }); entries.push_back({ j,j,w });
                    entries.push_back({ ref,j,-w }); entries.push_back({ j,ref,-w });
                }
            }
            for (int i : movable_ids) {
                for (const auto& fc : fixed_coords) {
                    entries.push_back({ i,i,w });
                    bx[i] += w * fc.first;
                    by[i] += w * fc.second;
                }
            }
        }
        coo_matrix A;
        A.n = n;
        A.nnz = static_cast<int>(entries.size());
        A.row.resize(A.nnz); A.col.resize(A.nnz); A.dat.resize(A.nnz);
        for (int idx = 0; idx < A.nnz; ++idx) {
            A.row[idx] = entries[idx].r; A.col[idx] = entries[idx].c; A.dat[idx] = entries[idx].v;
        }
        std::valarray<double> solX(0.0, n); std::valarray<double> solY(0.0, n);
        A.solve(bx, solX); A.solve(by, solY);
        for (int i = 0; i < n; ++i) {
            instance_pointers_[i]->setCoordinate((int)std::round(solX[i]), (int)std::round(solY[i]));
        }
    }
}