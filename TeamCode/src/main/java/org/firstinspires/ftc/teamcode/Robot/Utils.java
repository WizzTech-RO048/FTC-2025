package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import android.util.Pair;
import androidx.annotation.RequiresApi;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Predicate;
import java.util.function.Supplier;


public class Utils {
    public static double interpolate(double a, double b, double p, double exp) {
        double factor = Math.pow(1-p, exp);
        return a * factor + b * (1 - p);
    }

    public static boolean isDone(ScheduledFuture<?> f) {
        return f == null || f.isDone();
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(val, max));
    }

    public static ScheduledFuture<?> poll(ScheduledExecutorService scheduledExecutorService,
                                          Supplier<Boolean> fn,
                                          Runnable onEnd,
                                          long time,
                                          TimeUnit timeUnit) {
        return pollIndex(scheduledExecutorService, _i -> fn.get(), onEnd, time, timeUnit);
    }

    public static ScheduledFuture<?> pollIndex(ScheduledExecutorService scheduler,
                                               Predicate<Integer> fn,
                                               Runnable onEnd,
                                               long time,
                                               TimeUnit unit) {
        AtomicBoolean endCalled = new AtomicBoolean();
        AtomicInteger pollIndex = new AtomicInteger();

        ScheduledFuture<?> f = scheduler.scheduleAtFixedRate(() -> {
            if (!fn.test(pollIndex.getAndIncrement())) {
                return;
            }
            if (onEnd != null && endCalled.compareAndSet(false, true)) {
                onEnd.run();
            }
            throw new RuntimeException();
        }, 0, time, unit);

        return new ScheduledFuture<Object>() {
            @Override
            public long getDelay(TimeUnit unit) {
                return f.getDelay(unit);
            }

            @Override
            public int compareTo(Delayed o) {
                return f.compareTo(o);
            }

            @Override
            public boolean cancel(boolean mayInterruptIfRunning) {
                if (!f.cancel(mayInterruptIfRunning)) {
                    return false;
                }

                if (endCalled.compareAndSet(false, true)) {
                    onEnd.run();
                }

                return true;
            }

            @Override
            public boolean isCancelled() {
                return f.isCancelled();
            }

            @Override
            public boolean isDone() {
                return f.isDone();
            }

            @Override
            public Object get() throws ExecutionException, InterruptedException {
                return f.get();
            }

            @Override
            public Object get(long timeout, TimeUnit unit) throws ExecutionException, InterruptedException, TimeoutException {
                return f.get(timeout, unit);
            }
        };
    }


}