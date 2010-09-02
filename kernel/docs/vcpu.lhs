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

cmpPrio (_, _, vT1, _) (_, _, vT2, _) = vT1 `compare` vT2
vcpuId (id, _, _, _) = id
vcpuT (_, _, t, _) = t

schedule :: PCPU -> PCPU
schedule (curId, runnable, tcur, tprev) = (curId', runnable', tcur + tdelta', tcur)
  where
    tdelta = tcur - tprev
    runnable' = flip map runnable $ \ (vid, vb, vT, vR) ->
                  let (vR', time) = foldr (\ (t, b) (repls, time) ->
                                             if t <= tcur 
                                             then (repls, time + b)
                                             else ((t, b):repls, time))
                                          ([], 0)
                                          (if Just vid == curId
                                           then (tprev + vT, used):vR
                                           else vR)
                      vb' = max 0 (vb - tdelta)
                      used = vb - vb'
                  in
                    (vid, time + if Just vid == curId then vb' else vb, vT, vR')
    (curId', nextT) = case filter (\ (_, b, _, _) -> b > 0) runnable' of
                        [] -> (Nothing, 0)
                        vs -> ((Just . vcpuId) &&& vcpuT) (minimumBy cmpPrio vs)
    events = flip concatMap runnable' $ \ (vid, vb, vT, vR) ->
                if Just vid == curId' then [vb] else [] ++
                if vT <= nextT then map (subtract tcur . fst) vR else []
    tdelta' = if null events then 1 else minimum events

makePCPU :: [(Integer, Integer)] -> PCPU
makePCPU specs = (Nothing, zipWith (\ i (c, t) -> (i, c, t, [])) [0..] specs, 0, 0)

run :: PCPU -> [VCPUID]
run pcpu = concatMap (\ (cur, _, _, _) -> maybeToList cur) $ iterate schedule pcpu

test1 = makePCPU [(1, 5), (1, 6), (1, 7), (1, 8)]

\end{code}
