"""Choosing the order of deliveries (the optimisation at the heart of global planning).

The problem
-----------
The robot starts at `start`. Each object i must be picked up at its pickup place
and dropped at its exit's drop-off place. The robot can carry `capacity` objects
at once (1 for now). We want the order that takes the least total time.

A *trip* is: pick up a set of objects (at most `capacity`), then drive to their
exits and drop them. The mission is a sequence of trips, and it ends when the
last object is dropped (the robot does not have to come back).

With capacity 1 a trip is simply "pick object i, deliver object i", and the
problem is like a travelling-salesman problem over the objects: the cost of
doing j right after i is  time(exit of i -> object j) + pick + time(object j ->
exit of j) + place.

The exact solver (dynamic programming over subsets)
---------------------------------------------------
Trying every order costs n! (3.6 million orders for 10 objects). Instead, note
that once a set of objects has been delivered, all that matters for the rest of
the mission is *which* objects are done and *where the robot is now* (the exit
it last dropped at), not the order they were done in. So we keep, for every
state (done-set, robot place), only the cheapest way found to reach it:

    best[done | T][end] = min over trips T from robot place `loc` ending at exit `end`
                          of best[done][loc] + trip_time(loc, T, end)

and the answer is the cheapest best[all objects][any end]. This is the
Held-Karp algorithm generalised to trips. For capacity 1 it does about
n^2 * 2^n steps: 12 objects take well under a second in Python, while
brute force would need 479 million orders. It is exactly optimal.

Above `max_exact_objects` a heuristic (greedy plus local improvement) is used
instead and the plan is labelled "heuristic".

Assumption: within one trip the robot picks up all its objects before it drops
any of them. With capacity 1 this is no restriction at all.
"""

import itertools
import math
from dataclasses import dataclass, field


@dataclass
class SolverInput:
    start: int                 # place index where the robot is now
    pickup: list               # per object: place index of its pickup, or None if already carried
    drop: list                 # per object: place index of its exit drop-off
    leg_time: list             # leg_time[a][b]: driving time from place a to place b (s)
    pick_time: float = 20.0    # s per object picked up
    place_time: float = 8.0    # s per object dropped
    capacity: int = 1


@dataclass
class Trip:
    picks: list                # object indexes in pickup order (empty for objects already carried)
    drops: list                # [(drop place, [object indexes dropped there])] in driving order
    time: float = 0.0          # s for the whole trip, including picks and drops


@dataclass
class Solution:
    trips: list = field(default_factory=list)
    total_time: float = math.inf
    solver: str = 'exact'


class DeliverySolver:
    """Finds the fastest sequence of trips. See the module docstring."""

    def __init__(self, inp, max_exact_objects=12):
        self.inp = inp
        self.max_exact_objects = max_exact_objects
        self._trip_cache = {}

    # ------------------------------------------------------------ public
    def solve(self):
        inp = self.inp
        carried = [i for i, p in enumerate(inp.pickup) if p is None]
        waiting = [i for i, p in enumerate(inp.pickup) if p is not None]

        if not carried:
            return self._solve_waiting(inp.start, waiting)

        # Objects already in the gripper are delivered first, as their own trip
        # (there is nothing to pick up). Which exit that trip ends at affects
        # the rest of the mission, so try each ending and keep the best total.
        best = Solution()
        for end, (t, drops) in self._delivery_orders(inp.start, carried).items():
            rest = self._solve_waiting(end, waiting)
            total = t + rest.total_time
            if total < best.total_time:
                first = Trip(picks=[], drops=drops, time=t)
                best = Solution([first] + rest.trips, total, rest.solver)
        return best

    # ------------------------------------------------------------ trips
    def _delivery_orders(self, loc, objects):
        """Best way to drop `objects` (already carried) starting at place loc.

        Returns {end place: (time, drops)} with the cheapest drop order for each
        possible last exit. Objects going to the same exit are dropped together.
        """
        inp = self.inp
        groups = {}
        for i in objects:
            groups.setdefault(inp.drop[i], []).append(i)
        best = {}
        for order in itertools.permutations(groups):
            t, here = 0.0, loc
            for place in order:
                t += inp.leg_time[here][place] + inp.place_time * len(groups[place])
                here = place
            end = order[-1]
            if end not in best or t < best[end][0]:
                best[end] = (t, [(place, list(groups[place])) for place in order])
        return best

    def _trip_options(self, loc, objects):
        """All useful ways to do one trip from place loc with this set of objects.

        Returns {end place: (time, picks, drops)}: for each exit the trip could
        end at, the fastest pickup order and drop order. Cached, because the
        dynamic programme asks for the same (loc, set) many times.
        """
        key = (loc, objects)
        if key in self._trip_cache:
            return self._trip_cache[key]
        inp = self.inp
        best = {}
        for picks in itertools.permutations(objects):
            t, here = 0.0, loc
            for i in picks:
                t += inp.leg_time[here][inp.pickup[i]] + inp.pick_time
                here = inp.pickup[i]
            for end, (dt, drops) in self._delivery_orders(here, picks).items():
                total = t + dt
                if end not in best or total < best[end][0]:
                    best[end] = (total, list(picks), drops)
        self._trip_cache[key] = best
        return best

    # ------------------------------------------------------------ exact
    def _solve_waiting(self, loc0, objects):
        if not objects:
            return Solution([], 0.0, 'exact')
        if len(objects) > self.max_exact_objects:
            return self._solve_heuristic(loc0, objects)
        return self._solve_exact(loc0, objects)

    def _solve_exact(self, loc0, objects):
        n = len(objects)
        cap = max(1, self.inp.capacity)
        full = (1 << n) - 1

        # Every possible trip: a non-empty set of at most `cap` objects, as a bitmask
        # over positions in `objects`.
        trip_masks = []
        for size in range(1, min(cap, n) + 1):
            for combo in itertools.combinations(range(n), size):
                m = 0
                for k in combo:
                    m |= 1 << k
                trip_masks.append((m, tuple(objects[k] for k in combo)))

        # best[mask] maps robot place -> (time so far, previous mask, previous place, trip).
        best = [None] * (1 << n)
        best[0] = {loc0: (0.0, None, None, None)}
        # Masks only ever grow, so visiting them in increasing order means every
        # state is final before it is extended.
        for mask in range(1 << n):
            states = best[mask]
            if not states:
                continue
            for tmask, tobjs in trip_masks:
                if tmask & mask:
                    continue  # some of these objects are already delivered
                nmask = mask | tmask
                for loc, (t0, _, _, _) in states.items():
                    for end, (tt, picks, drops) in self._trip_options(loc, tobjs).items():
                        total = t0 + tt
                        if best[nmask] is None:
                            best[nmask] = {}
                        cur = best[nmask].get(end)
                        if cur is None or total < cur[0]:
                            best[nmask][end] = (total, mask, loc, Trip(picks, drops, tt))

        # Cheapest way to have delivered everything, then walk the back-links.
        end, (total, _, _, _) = min(best[full].items(), key=lambda kv: kv[1][0])
        trips = []
        mask, loc = full, end
        while mask:
            _, pmask, ploc, trip = best[mask][loc]
            trips.append(trip)
            mask, loc = pmask, ploc
        trips.reverse()
        return Solution(trips, total, 'exact')

    # ------------------------------------------------------------ heuristic
    def _sequence_time(self, loc, groups):
        """Time of doing the trips `groups` (tuples of objects) in this order."""
        trips, total = [], 0.0
        for g in groups:
            options = self._trip_options(loc, tuple(g))
            end, (t, picks, drops) = min(options.items(), key=lambda kv: kv[1][0])
            trips.append(Trip(picks, drops, t))
            total += t
            loc = end
        return total, trips

    def _solve_heuristic(self, loc0, objects):
        """Good (not guaranteed optimal) plan for many objects.

        1. Group objects into trips: objects for the same exit go together, up to
           the capacity (with capacity 1 every object is its own trip).
        2. Order the trips greedily (cheapest next trip first).
        3. Improve: keep swapping any two trips or moving one trip to another
           position while that makes the total shorter.
        """
        cap = max(1, self.inp.capacity)
        by_exit = {}
        for i in objects:
            by_exit.setdefault(self.inp.drop[i], []).append(i)
        groups = []
        for objs in by_exit.values():
            for k in range(0, len(objs), cap):
                groups.append(tuple(objs[k:k + cap]))

        order, loc, left = [], loc0, list(groups)
        while left:
            g = min(left, key=lambda g: min(t for t, _, _ in
                                            self._trip_options(loc, g).values()))
            left.remove(g)
            order.append(g)
            options = self._trip_options(loc, g)
            loc = min(options.items(), key=lambda kv: kv[1][0])[0]

        best_t, _ = self._sequence_time(loc0, order)
        improved = True
        while improved:
            improved = False
            for a in range(len(order)):
                for b in range(len(order)):
                    if a == b:
                        continue
                    for move in ('swap', 'insert'):
                        cand = list(order)
                        if move == 'swap':
                            cand[a], cand[b] = cand[b], cand[a]
                        else:
                            cand.insert(b, cand.pop(a))
                        t, _ = self._sequence_time(loc0, cand)
                        if t < best_t - 1e-9:
                            order, best_t, improved = cand, t, True
        total, trips = self._sequence_time(loc0, order)
        return Solution(trips, total, 'heuristic')


def brute_force_time(inp):
    """Fastest total time by trying *every* plan. Only for checking the solver.

    Independent of DeliverySolver: every order of the objects, every way of
    cutting that order into consecutive trips of at most `capacity`, and every
    drop order within each trip. Grows like n! * 2^n, so keep n <= 6.
    Objects already carried are not supported here.
    """
    n = len(inp.pickup)
    cap = max(1, inp.capacity)
    best = math.inf

    def cuts(k):
        # All ways to split k items into consecutive chunks of size 1..cap.
        if k == 0:
            yield []
            return
        for size in range(1, min(cap, k) + 1):
            for rest in cuts(k - size):
                yield [size] + rest

    for order in itertools.permutations(range(n)):
        for sizes in cuts(n):
            # Each trip's best drop order depends on where the trip starts, so
            # keep all reachable (location, time) pairs.
            states = {inp.start: 0.0}
            pos = 0
            for size in sizes:
                chunk = order[pos:pos + size]
                pos += size
                new_states = {}
                for loc, t in states.items():
                    here, tt = loc, t
                    for i in chunk:
                        tt += inp.leg_time[here][inp.pickup[i]] + inp.pick_time
                        here = inp.pickup[i]
                    groups = {}
                    for i in chunk:
                        groups.setdefault(inp.drop[i], []).append(i)
                    for exits in itertools.permutations(groups):
                        h, t2 = here, tt
                        for e in exits:
                            t2 += inp.leg_time[h][e] + inp.place_time * len(groups[e])
                            h = e
                        if t2 < new_states.get(h, math.inf):
                            new_states[h] = t2
                states = new_states
            best = min(best, min(states.values()))
    return best
