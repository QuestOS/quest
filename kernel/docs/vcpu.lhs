\begin{code}
module VCPU where
import Control.Arrow
import Data.List
import Data.Maybe

type VCPUID = Int
-- (current, runnable, tcur, tprev)
type PCPU = (Maybe VCPUID, [VCPU], Integer, Integer)
-- (id, budget, period, replenishments)
type VCPU = (VCPUID, Integer, Integer, [Replenishment])
-- (time, budget)
type Replenishment = (Integer, Integer)

cmpId (id1, _, _, _) (id2, _, _, _) = id1 `compare` id2
eqId v1 v2 = cmpId v1 v2 == EQ
cmpPrio (_, _, vT1, _) (_, _, vT2, _) = vT1 `compare` vT2
vcpuId (id, _, _, _) = id
vcpuT (_, _, t, _) = t

schedule :: PCPU -> PCPU
schedule (curId, runnable, tcur, tprev) = (nextId, runnable', tcur + tdelta', tcur)
  where
    tdelta = tcur - tprev

    updateRunnable (vid, vb, vT, vR)
      -- working on current VCPU
      | Just vid == curId = (vid, time + curvb', vT, (tprev + vT, used):vR')
      -- working on non-running VCPU
      | otherwise         = (vid, time + vb, vT, vR')
      where
        -- accumulate and remove pending replenishments  
        checkReplenishment (t, b) (repls, time)
          | t <= tcur = (repls, time + b)
          | otherwise = ((t, b):repls, time)
        (vR', time) = foldr checkReplenishment ([], 0) vR
        -- budget variables for use if vid is current VCPU
        curvb' = max 0 (vb - tdelta)
        used = vb - curvb'

    runnable' = map updateRunnable runnable

    -- pick highest priority VCPU with non-zero budget  
    (nextId, nextT) = case filter (\ (_, b, _, _) -> b > 0) runnable' of
                        [] -> (Nothing, 0)
                        vs -> ((Just . vcpuId) &&& vcpuT) (minimumBy cmpPrio vs)

    -- compute time deltas for higher priority VCPU replenishments,
    -- and also its budget if it is the next VCPU to run
    vcpuDeltas (vid, vb, vT, vR) =
      if Just vid == nextId then [vb] else [] ++
      if vT <= nextT then map (subtract tcur . fst) vR else []

    deltas = concatMap vcpuDeltas runnable'
    -- new tdelta is the smallest delta
    tdelta' = if null deltas then 1 else minimum deltas

makePCPU :: [(Integer, Integer)] -> PCPU
makePCPU specs = (Nothing, zipWith (\ i (c, t) -> (i, c, t, [])) [0..] specs, 0, 0)

run :: PCPU -> [VCPUID]
run pcpu = concatMap (\ (cur, _, _, _) -> maybeToList cur) $ iterate schedule pcpu

usage :: Int -> PCPU -> [(Maybe VCPUID, Float)]
usage n pcpu = 
  map ((vcpuId . head) &&& ((/ fromIntegral n) . fromIntegral . length))
      . groupBy eqId . sortBy cmpId
      . take n $ iterate schedule pcpu

test1 = makePCPU [(1, 5), (1, 6), (1, 7), (1, 8)]
test2 = makePCPU [(1, 5), (1, 6), (1, 7), (1, 25)]
test3 = makePCPU [(1, 4), (1, 4), (1, 4), (1, 4)]
test4 = makePCPU [(1, 3), (1, 4), (1, 5)]
test5 = makePCPU [(1, 3), (1, 4), (2, 5)]
test6 = makePCPU [(1, 2), (1, 3), (1, 4)]

\end{code}
