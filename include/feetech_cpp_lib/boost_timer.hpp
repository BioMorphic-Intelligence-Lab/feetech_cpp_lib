#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#ifndef BOOST_TIMER_HPP_
#define BOOST_TIMER_HPP_

/* Creates a timer that calls the callback function at a given frequency in a separate thread.
 *
 * The timer is fixed-RATE: each tick is scheduled relative to the previous tick's
 * target time (not relative to when the callback finished). This prevents the
 * callback's (variable) execution time from accumulating as drift, which would
 * otherwise make the effective frequency non-constant. If a callback overruns its
 * period, the timer resynchronizes instead of trying to "catch up" indefinitely. */
class BoostTimer {
public:
    BoostTimer(double frequency, std::function<void()> callback)
        : io_service_(), timer_(io_service_), callback_(callback),
          period_(std::chrono::microseconds(static_cast<long>(1.0e6 / frequency))) {
        thread_ = std::thread([this]() { io_service_.run(); });
        timer_.expires_after(period_);
        schedule();
    }

    ~BoostTimer() {
        io_service_.stop();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    void schedule() {
        timer_.async_wait([this](const boost::system::error_code& error_code) {
            if (!error_code) {
                callback_();

                // Schedule the next tick relative to the previous target time so
                // execution time does not drift the rate. If we have fallen behind
                // by more than one period, resync to now to avoid a burst of
                // back-to-back callbacks.
                auto next = timer_.expiry() + period_;
                if (next < std::chrono::steady_clock::now())
                    timer_.expires_after(period_);
                else
                    timer_.expires_at(next);

                schedule();
            }
        });
    }

    boost::asio::io_service io_service_;
    boost::asio::steady_timer timer_;
    std::function<void()> callback_;
    std::chrono::microseconds period_;
    std::thread thread_;
};

#endif