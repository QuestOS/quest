\begin{code}
module VCPU where
import Control.Arrow
import Data.List
import Data.Maybe

type VCPUID = Int
-- (current, runnable, tcur, tprev)
type PCPU = (Maybe VCPUID, [VCPU], Integer, Integer)
-- (id, budget, period, replenishments, activation)
type VCPU = (VCPUID, Integer, Integer, [Replenishment], Integer)
-- (time, budget)
type Replenishment = (Integer, Integer)

cmpId (id1, _, _, _) (id2, _, _, _) = id1 `compare` id2
eqId v1 v2 = cmpId v1 v2 == EQ
pcpuCur (id, _, _, _) = id
findPrio m_id runnable = do
  id <- m_id
  v  <- find ((== id) . vcpuId) runnable
  return $ vcpuT v

cmpPrio (_, _, vT1, _, _) (_, _, vT2, _, _) = vT1 `compare` vT2
vcpuId (id, _, _, _, _) = id
vcpuT (_, _, t, _, _) = t
vcpuBudget (_, b, _, _, _) = b

nextMul vT tcur = tcur + vT - tcur `mod` vT

-- check if changing from 'prev' to 'next' priority causes task with
-- 'prio' to become active
activated _ _ Nothing = False
activated m_prev prio (Just next) = prio >= next && maybe True (> prio) m_prev

schedule :: PCPU -> PCPU
schedule (curId, runnable, tcur, tprev) = (nextId, runnable'', tcur + tdelta', tcur)
  where
    tdelta = tcur - tprev
    prevPrio = findPrio curId runnable

    updateRunnable (vid, vb, vT, vR, act)
      -- working on current VCPU
      | Just vid == curId = (vid, time + curvb', vT, vR', act')
      -- working on non-running VCPU
      | otherwise         = (vid, time + vb, vT, vR', act')
      where
        -- accumulate and remove pending replenishments  
        checkReplenishment (t, b) (repls, time)
          | t <= tcur = (repls, time + b)
          | otherwise = ((t, b):repls, time)
        (vR', time) = foldr checkReplenishment ([], 0) $
                        if Just vid == curId
                        -- include current replenishment
                        then (rt, used):vR
                        else vR
        act' = if time > 0 then tcur else act
        -- budget variables for use if vid is current VCPU
        curvb' = max 0 (vb - tdelta)
        used = vb - curvb'
        -- compute replenishment time
        rt = act + vT
        -- rt = nextMul vT tcur

    runnable' = map updateRunnable runnable

    -- pick highest priority VCPU with non-zero budget  
    (nextId, nextT) = case filter ((> 0) . vcpuBudget) runnable' of
                        [] -> (Nothing, 0)
                        vs -> ((Just . vcpuId) &&& vcpuT) (minimumBy cmpPrio vs)

    -- compute time deltas for higher priority VCPU replenishments,
    -- and also its budget if it is the next VCPU to run
    vcpuDeltas (vid, vb, vT, vR, act) =
      if Just vid == nextId then [vb] else [] ++
      if vT <= nextT then map (subtract tcur . fst) vR else []

    deltas = concatMap vcpuDeltas runnable'
    -- new tdelta is the smallest delta
    tdelta' = if null deltas then 1 else minimum deltas

    -- set activation times for tasks that became active
    nextPrio = const nextT `fmap` nextId
    runnable'' = flip map runnable' $ \ (vid, vb, vT, vR, act) ->
                   if vb > 0 && activated prevPrio vT nextPrio then
                     (vid, vb, vT, vR, tcur)
                   else
                     (vid, vb, vT, vR, act)


makePCPU :: [(Integer, Integer)] -> PCPU
makePCPU specs = (Nothing, zipWith (\ i (c, t) -> (i, c, t, [], 0)) [0..] 
                                   specs, 0, 0)

run :: PCPU -> [VCPUID]
run pcpu = concatMap (\ (cur, _, _, _) -> maybeToList cur) $ iterate schedule pcpu

ranFor (_, _, tcur, tprev) = fromIntegral $ tcur - tprev

usage :: Int -> PCPU -> [(Maybe VCPUID, Float)]
usage n pcpu = 
  map ((pcpuCur . head) &&& ((/ fromIntegral n) . fromIntegral . length))
      . groupBy eqId . sortBy cmpId
      . take n . concatMap (\ p -> replicate (ranFor p) p) $ iterate schedule pcpu

utilization :: PCPU -> (Double, Double)
utilization (_, runnable, _, _) =
  ( sum . map (uncurry (/))
        $ map (fromIntegral . vcpuBudget &&& fromIntegral . vcpuT) runnable
  , n * (2**(1 / n) - 1) )
  where n = fromIntegral $ length runnable

test1 = makePCPU [(1, 5), (1, 6), (1, 7), (1, 8)]
test2 = makePCPU [(1, 5), (1, 6), (1, 7), (1, 25)]
test3 = makePCPU [(1, 4), (1, 4), (1, 4), (1, 4)]
test4 = makePCPU [(1, 3), (1, 4), (1, 5)]
test5 = makePCPU [(1, 3), (1, 4), (2, 5)]
test6 = makePCPU [(1, 2), (1, 3), (1, 4)]
test7 = makePCPU [(1, 5), (1, 5), (1, 5), (1, 5)]
test8 = makePCPU [(1, 4), (2, 5), (3, 10)]
test9 = makePCPU [(10, 200), (20, 50), (49, 200)]

\end{code}
