/**
 * @file src/latency.cc
 * @brief 实测 CPU 各级缓存（L1、L2、L3/LLC）及主存（DRAM）的访问延迟（C++17）
 *        支持通过命令行开关逐项启停抗干扰策略，直观对比各项硬件机制对测量结果的影响。
 *
 * ============================================================================
 * 一、核心测试思路
 * ============================================================================
 * 1. 指针追踪（Pointer Chasing）构造严格的数据依赖链：
 *    现代 CPU 具备强大的乱序执行（Out-of-Order Execution）和内存级并行（MLP）能力。
 *    若采用普通数组顺序遍历或并发多链访问，CPU 会同时在流水线中发出十几个访存请求，
 *    此时测出的是【内存吞吐带宽/并发分摊耗时】而非【单次访存延迟】。
 *    为测量真实的 Load-to-Use 延迟，本程序将内存划分为槽位节点并串行执行：
 *        p = *p;
 *    由于下一次内存读取的地址完全取决于上一次读取返回的数据，CPU 被迫串行等待每次
 *    Load 完成，总耗时除以追踪步数即为单次访存延迟。
 *    -> 可通过 `--no-dep`（或 `--mlp 8`）关闭单链串行约束，观察 MLP 将主存耗时拉低数倍。
 *
 * 2. 工作集大小扫描（Working Set Size Sweep）打出延迟阶梯：
 *    将连续测试的工作集大小从 4 KB 按 2 倍步进扩展至 256 MB（可通过参数调整）：
 *      - 工作集 <= L1D 容量       ：几乎全部命中 L1，呈现第一级低延迟平台（~0.7-1.5 ns）
 *      - L1D < 工作集 <= L2 容量  ：溢出 L1 但命中 L2，跳升至第二级平台（~3-5 ns）
 *      - L2  < 工作集 <= L3 容量  ：主要命中 L3（LLC），呈现第三级平台（~10-25 ns）
 *      - 工作集 >> L3 容量        ：缓存命中率趋近于 0，测得主存延迟（~60-120 ns）
 *    每一级延迟平台向下一级跃升的拐点，即对应各级缓存的物理容量边界。
 *
 * ============================================================================
 * 二、防干扰工程细节与对照实验开关（均可在运行时启停）
 * ============================================================================
 * 1. 击败硬件流/步长预取器（默认 `--pattern sattolo --stride 128`）：
 *    - 顺序访问会被流预取器（Stream Prefetcher）提前拉入缓存。
 *      -> 对照实验：`--no-random`（等价 `--pattern sequential`），256 MB 大内存仍会测出
 *         ~1-3 ns 的极低假延迟。
 *    - 普通 Fisher-Yates 洗牌平均分裂成约 ln(N) 个不相交子环，一旦起点落入局部小环就会
 *      永远命中缓存。
 *      -> 对照实验：`--pattern fisher-yates`，可观察到大工作集延迟随机暴跌回 L1/L2 水平。
 *    - 步长默认设为 128B：既匹配 Apple Silicon 128B 缓存行，又规避 x86 相邻行预取（2x64B）。
 *      -> 对照实验：`--stride 64`（同缓存行附带预取拉低延迟）或 `--stride 256`（只打中偶
 *         数号 Cache Set，导致 L1/L2 有效容量直接腰斩提前溢出）。
 *
 * 2. 控制 TLB Miss（页表遍历 Page Walk）开销（默认开启大页，关闭用 `--no-hugepage`）：
 *    - 运行时通过 sysconf(_SC_PAGESIZE) 获取真实基础页大小，在 Linux 下动态读取 THP 大页
 *      尺寸（2MB / 32MB / 512MB），当工作集达到大页大小时按大页对齐并启用 MADV_HUGEPAGE。
 *      -> 对照实验：Linux 下传入 `--no-hugepage`（显式调用 MADV_NOHUGEPAGE），可观察到大
 *         工作集因跨页触发 STLB Miss + Page Walk 而额外增加 15~40 ns 延迟。
 *
 * 3. 消除循环指令开销与编译器优化（默认 64 路展开，关闭用 `--no-unroll`）：
 *    - 利用 C++17 编译期递归模板将指针解引用展开 64 次，摊薄分支与计数开销；结果写入
 *      volatile 全局槽位防止 -O2/-O3 死代码消除。
 *      -> 对照实验：`--no-unroll` 禁用循环展开，观察单步循环分支在高频 L1 下的额外开销。
 *
 * 4. 数据依赖预取器（DMP）屏蔽实验（默认裸指针，开启屏蔽用 `--mask-dmp`）：
 *    - Apple Silicon 与部分新款 x86 核心具备 DMP，会扫描缓存行中形似虚拟地址的数据提前预取。
 *      -> 对照实验：`--mask-dmp` 将内存中的指针异或掩码加密存储，屏蔽 DMP 扫描（同时可观察
 *         每步多出一条 XOR 指令给 L1 带来的 ~1 cycle 算术延迟增量）。
 *
 * 5. 核心绑定、异构核调度与预热（`--no-pin` / `--ecore` / `--no-warmup` / `--no-cpu-warmup`）：
 *    - 默认锁定单核（Linux）/ 性能核 P-core（macOS），执行 CPU 升频预热与工作集缓存预热。
 *      -> 对照实验：`--ecore`（macOS 强制调度至能效核 E-core，可观察 L1/L2 容量拐点整体缩小）；
 *         `--no-warmup`（测试前冲刷缓存并仅测单趟冷访问，观察小工作集因 Cold Miss 飙升至数十 ns）；
 *         `--no-cpu-warmup`（跳过 CPU 升频预热）；`--trials 1`（关闭多轮中位数抗毛刺）。
 *
 * ============================================================================
 * 三、编译与运行示例
 * ============================================================================
 *   make run-latency
 *   make run-latency ARGS="--help"
 *   make run-latency ARGS="--no-random"
 *   make run-latency ARGS="--stride 256"
 *   make run-latency ARGS="--no-dep"
 *   make run-latency ARGS="--ecore"
 *   make run-latency ARGS="--no-warmup"
 */

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <unistd.h>
#endif

#if defined(__linux__)
#include <sched.h>
#include <sys/mman.h>
#elif defined(__APPLE__)
#include <pthread.h>
#include <pthread/qos.h>
#include <sys/sysctl.h>
#endif

namespace {

inline constexpr std::size_t kDefaultStride = 128;
inline constexpr std::size_t kUnrollFactor = 64;
inline constexpr std::size_t kMaxMlpChains = 8;
inline constexpr std::size_t kColdEvictBytes = 64 * 1024 * 1024;  // 64 MB 冲刷缓冲区
inline constexpr std::uintptr_t kDmpXorMask = static_cast<std::uintptr_t>(0x5a5a5a5a5a5a5a5aULL);

enum class AccessPattern {
    kSattolo,      // 严格单哈密顿环随机排列（默认，防预取且遍历全集）
    kFisherYates,  // 普通随机置换（易分裂出局部小环导致假命中）
    kSequential,   // 顺序步进访问（触发硬件流预取器）
};

enum class ThreadPolicy {
    kPinPerf,     // Linux 绑单核 / macOS 锁定 P-Core（默认）
    kDefault,     // 交由 OS 默认调度（不绑核、不设 QoS）
    kEfficiency,  // macOS 强制调度至 E-Core / Linux 绑末尾逻辑核
};

struct Config {
    std::size_t min_kb = 4;
    std::size_t max_kb = 262'144;  // 默认测至 256 MB
    std::size_t trials = 3;
    std::size_t stride = kDefaultStride;
    std::size_t mlp_chains = 1;  // 1 | 2 | 4 | 8

    AccessPattern pattern = AccessPattern::kSattolo;
    ThreadPolicy thread_policy = ThreadPolicy::kPinPerf;
    bool use_hugepage = true;
    bool unroll_loop = true;
    bool cache_warmup = true;
    bool cpu_warmup = true;
    bool mask_dmp = false;
};

constexpr bool is_power_of_two(std::size_t x) noexcept {
    return x > 0 && ((x & (x - 1)) == 0);
}

constexpr bool is_valid_alignment(std::size_t align) noexcept {
    return align >= sizeof(void*) && is_power_of_two(align);
}

struct PageConfig {
    std::size_t base_page_bytes = 4096;
    std::size_t huge_page_bytes = 0;  // 0 表示当前系统未启用/不支持 THP 动态探测
};

#if defined(__linux__)
std::size_t detect_linux_huge_page_bytes() noexcept {
    {
        std::ifstream thp_file("/sys/kernel/mm/transparent_hugepage/hpage_pmd_size");
        std::size_t bytes = 0;
        if (thp_file >> bytes && is_valid_alignment(bytes)) {
            return bytes;
        }
    }
    {
        std::ifstream meminfo("/proc/meminfo");
        std::string key;
        std::size_t kb = 0;
        std::string unit;
        while (meminfo >> key) {
            if (key == "Hugepagesize:") {
                if ((meminfo >> kb >> unit) && is_valid_alignment(kb * 1024)) {
                    return kb * 1024;
                }
                break;
            }
            meminfo.ignore(4096, '\n');
        }
    }
    return 0;
}
#endif

PageConfig detect_page_config() noexcept {
    PageConfig cfg{};
#if defined(__unix__) || defined(__APPLE__)
    const long ps = ::sysconf(_SC_PAGESIZE);
    if (ps > 0 && is_valid_alignment(static_cast<std::size_t>(ps))) {
        cfg.base_page_bytes = static_cast<std::size_t>(ps);
    }
#endif
#if defined(__linux__)
    const std::size_t hp = detect_linux_huge_page_bytes();
    if (hp > cfg.base_page_bytes) {
        cfg.huge_page_bytes = hp;
    }
#endif
    return cfg;
}

// 防止编译器优化掉只读指针追踪循环与缓存冲刷读取
volatile std::uintptr_t g_sink = 0;

struct AlignedDeleter {
    void operator()(unsigned char* ptr) const noexcept { std::free(ptr); }
};

using AlignedByteBuffer = std::unique_ptr<unsigned char[], AlignedDeleter>;

AlignedByteBuffer allocate_buffer(std::size_t alloc_bytes,
                                  std::size_t min_align,
                                  const PageConfig& page_cfg,
                                  bool allow_hugepage) {
    const std::size_t base_align = std::max(page_cfg.base_page_bytes, min_align);
    const bool try_huge = allow_hugepage && page_cfg.huge_page_bytes > 0 &&
                          alloc_bytes >= page_cfg.huge_page_bytes;

    void* raw = nullptr;
    if (try_huge &&
        posix_memalign(&raw, std::max(page_cfg.huge_page_bytes, min_align), alloc_bytes) == 0) {
#if defined(__linux__) && defined(MADV_HUGEPAGE)
        (void)madvise(raw, alloc_bytes, MADV_HUGEPAGE);
#endif
    } else {
        if (posix_memalign(&raw, base_align, alloc_bytes) != 0) {
            throw std::bad_alloc{};
        }
#if defined(__linux__) && defined(MADV_NOHUGEPAGE)
        if (!allow_hugepage) {
            // 显式禁止内核自动合并透明大页（即使系统 THP 设为 always 也能真实测出页表开销）
            (void)madvise(raw, alloc_bytes, MADV_NOHUGEPAGE);
        }
#endif
    }

    std::memset(raw, 0, alloc_bytes);
    return AlignedByteBuffer{static_cast<unsigned char*>(raw)};
}

inline std::uintptr_t& slot_ref(unsigned char* base,
                                std::size_t idx,
                                std::size_t stride) noexcept {
    return *reinterpret_cast<std::uintptr_t*>(base + idx * stride);
}

inline std::uintptr_t slot_addr(const unsigned char* base,
                                std::size_t idx,
                                std::size_t stride) noexcept {
    return reinterpret_cast<std::uintptr_t>(base + idx * stride);
}

// 根据指定的访问模式初始化指针链表（并可选对指针做异或掩码加密以屏蔽硬件 DMP）
void build_pointer_chain(unsigned char* base,
                         std::size_t n,
                         std::size_t stride,
                         AccessPattern pattern,
                         bool mask_dmp) noexcept {
    if (pattern == AccessPattern::kSequential) {
        for (std::size_t i = 0; i < n; ++i) {
            slot_ref(base, i, stride) = slot_addr(base, (i + 1) % n, stride);
        }
    } else {
        for (std::size_t i = 0; i < n; ++i) {
            slot_ref(base, i, stride) = slot_addr(base, i, stride);
        }
        if (n > 1) {
            std::mt19937_64 rng{0x9e3779b97f4a7c15ULL ^ n};
            if (pattern == AccessPattern::kSattolo) {
                // Sattolo：j 严格小于 i，保证生成覆盖全部 N 个节点的单一哈密顿环
                for (std::size_t i = n - 1; i > 0; --i) {
                    std::uniform_int_distribution<std::size_t> dist(0, i - 1);
                    std::swap(slot_ref(base, i, stride), slot_ref(base, dist(rng), stride));
                }
            } else {
                // Fisher-Yates：j 可取到 i，随机置换会分裂为多个不相交子环
                for (std::size_t i = n - 1; i > 0; --i) {
                    std::uniform_int_distribution<std::size_t> dist(0, i);
                    std::swap(slot_ref(base, i, stride), slot_ref(base, dist(rng), stride));
                }
            }
        }
    }

    if (mask_dmp) {
        for (std::size_t i = 0; i < n; ++i) {
            slot_ref(base, i, stride) ^= kDmpXorMask;
        }
    }
}

template <bool MaskDmp>
#if defined(__GNUC__) || defined(__clang__)
[[gnu::always_inline]]
#endif
inline std::uintptr_t step_once(std::uintptr_t p) noexcept {
    const std::uintptr_t raw_next = *reinterpret_cast<const std::uintptr_t*>(p);
    if constexpr (MaskDmp) {
        return raw_next ^ kDmpXorMask;
    } else {
        return raw_next;
    }
}

// 编译期递归展开 Unroll 次单链串行读取
template <std::size_t Unroll, bool MaskDmp>
#if defined(__GNUC__) || defined(__clang__)
[[gnu::always_inline]]
#endif
inline std::uintptr_t chase_unrolled(std::uintptr_t p) noexcept {
    if constexpr (Unroll == 0) {
        return p;
    } else if constexpr (Unroll == 1) {
        return step_once<MaskDmp>(p);
    } else {
        constexpr std::size_t kHalf = Unroll / 2;
        return chase_unrolled<Unroll - kHalf, MaskDmp>(
            chase_unrolled<kHalf, MaskDmp>(p));
    }
}

// 多链并发（MLP）单轮步进：同时推进 Chains 条相互独立的指针链，暴露乱序执行内存级并行
template <std::size_t Chains, std::size_t StepsPerChain, bool MaskDmp>
#if defined(__GNUC__) || defined(__clang__)
[[gnu::always_inline]]
#endif
inline void chase_mlp_batch(std::array<std::uintptr_t, Chains>& chains) noexcept {
    for (std::size_t s = 0; s < StepsPerChain; ++s) {
        for (std::size_t c = 0; c < Chains; ++c) {
            chains[c] = step_once<MaskDmp>(chains[c]);
        }
    }
}

int configure_thread_execution(ThreadPolicy policy) noexcept {
    if (policy == ThreadPolicy::kDefault) {
        return -1;
    }
#if defined(__linux__)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    int cpu = 0;
    if (policy == ThreadPolicy::kEfficiency) {
        const long nprocs = ::sysconf(_SC_NPROCESSORS_ONLN);
        cpu = (nprocs > 1) ? static_cast<int>(nprocs - 1) : 0;
    } else {
        cpu = sched_getcpu();
        if (cpu < 0) cpu = 0;
    }
    CPU_SET(cpu, &cpuset);
    (void)sched_setaffinity(0, sizeof(cpuset), &cpuset);
    return cpu;
#elif defined(__APPLE__)
    if (policy == ThreadPolicy::kEfficiency) {
        (void)pthread_set_qos_class_self_np(QOS_CLASS_BACKGROUND, 0);
    } else {
        (void)pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
    }
    return -1;
#else
    return -1;
#endif
}

void warmup_cpu_frequency() noexcept {
    volatile std::uint64_t acc = 0;
    for (std::uint64_t i = 0; i < 120'000'000ULL; ++i) {
        acc += i;
    }
}

// 通过顺序读取大块无关缓冲区将待测工作集从各级缓存中挤出（用于 --no-warmup 冷访问实验）
void evict_caches(const unsigned char* thrash_buf, std::size_t bytes) noexcept {
    std::uintptr_t sum = 0;
    for (std::size_t offset = 0; offset < bytes; offset += 64) {
        sum += thrash_buf[offset];
    }
    g_sink = g_sink ^ sum;
}

template <bool MaskDmp>
double run_single_chain_trials(std::uintptr_t start,
                               std::size_t num_nodes,
                               std::size_t min_steps,
                               const Config& cfg,
                               const unsigned char* thrash_buf) {
    std::uintptr_t p = start;

    if (cfg.cache_warmup) {
        const std::size_t warmup_steps = std::max<std::size_t>(num_nodes, 65'536);
        for (std::size_t i = 0; i < warmup_steps; ++i) {
            p = step_once<MaskDmp>(p);
        }
    }

    // 若禁用缓存预热，仅跑单趟 num_nodes 步，防止上千万次热命中把首次冷未命中稀释掉
    const std::size_t target_steps =
        cfg.cache_warmup ? std::max(min_steps, num_nodes) : std::max<std::size_t>(num_nodes, 64);

    std::vector<double> samples;
    samples.reserve(cfg.trials);

    for (std::size_t t = 0; t < cfg.trials; ++t) {
        if (!cfg.cache_warmup && thrash_buf != nullptr) {
            evict_caches(thrash_buf, kColdEvictBytes);
            p = start;
        }

        if (cfg.unroll_loop) {
            const std::size_t loops = (target_steps + kUnrollFactor - 1) / kUnrollFactor;
            const std::size_t actual_steps = loops * kUnrollFactor;

            const auto t0 = std::chrono::steady_clock::now();
            for (std::size_t i = 0; i < loops; ++i) {
                p = chase_unrolled<kUnrollFactor, MaskDmp>(p);
            }
            const auto t1 = std::chrono::steady_clock::now();

            const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
            samples.push_back(ns / static_cast<double>(actual_steps));
        } else {
            const auto t0 = std::chrono::steady_clock::now();
#if defined(__clang__)
#pragma clang loop unroll(disable)
#elif defined(__GNUC__)
#pragma GCC unroll 1
#endif
            for (std::size_t i = 0; i < target_steps; ++i) {
                p = step_once<MaskDmp>(p);
            }
            const auto t1 = std::chrono::steady_clock::now();

            const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
            samples.push_back(ns / static_cast<double>(target_steps));
        }
    }

    g_sink = p;
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

template <std::size_t Chains, bool MaskDmp>
double run_mlp_trials(std::uintptr_t start,
                      std::size_t num_nodes,
                      std::size_t min_steps,
                      const Config& cfg,
                      const unsigned char* thrash_buf) {
    // 将 Chains 条并发链的起点沿环等距错开，避免后一条链直接命中前一条链刚拉回的缓存行
    std::array<std::uintptr_t, Chains> init_chains{};
    {
        std::uintptr_t cur = start;
        const std::size_t spacing = std::max<std::size_t>(1, num_nodes / Chains);
        for (std::size_t c = 0; c < Chains; ++c) {
            init_chains[c] = cur;
            for (std::size_t s = 0; s < spacing; ++s) {
                cur = step_once<MaskDmp>(cur);
            }
        }
    }

    auto chains = init_chains;
    if (cfg.cache_warmup) {
        const std::size_t warmup_iters = std::max<std::size_t>(num_nodes / Chains, 16'384);
        for (std::size_t i = 0; i < warmup_iters; ++i) {
            chase_mlp_batch<Chains, 1, MaskDmp>(chains);
        }
    }

    constexpr std::size_t kStepsPerChain = kUnrollFactor / Chains;
    constexpr std::size_t kLoadsPerBatch = Chains * kStepsPerChain;
    const std::size_t target_loads =
        cfg.cache_warmup ? std::max(min_steps, num_nodes)
                         : std::max<std::size_t>(num_nodes, kLoadsPerBatch);
    const std::size_t loops = (target_loads + kLoadsPerBatch - 1) / kLoadsPerBatch;
    const std::size_t actual_loads = loops * kLoadsPerBatch;

    std::vector<double> samples;
    samples.reserve(cfg.trials);

    for (std::size_t t = 0; t < cfg.trials; ++t) {
        if (!cfg.cache_warmup && thrash_buf != nullptr) {
            evict_caches(thrash_buf, kColdEvictBytes);
            chains = init_chains;
        }

        const auto t0 = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < loops; ++i) {
            chase_mlp_batch<Chains, kStepsPerChain, MaskDmp>(chains);
        }
        const auto t1 = std::chrono::steady_clock::now();

        const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        samples.push_back(ns / static_cast<double>(actual_loads));
    }

    std::uintptr_t acc = 0;
    for (std::uintptr_t c : chains) acc ^= c;
    g_sink = acc;

    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

template <bool MaskDmp>
double dispatch_measurement(std::uintptr_t start,
                            std::size_t num_nodes,
                            std::size_t min_steps,
                            const Config& cfg,
                            const unsigned char* thrash_buf) {
    switch (cfg.mlp_chains) {
        case 2:
            return run_mlp_trials<2, MaskDmp>(start, num_nodes, min_steps, cfg, thrash_buf);
        case 4:
            return run_mlp_trials<4, MaskDmp>(start, num_nodes, min_steps, cfg, thrash_buf);
        case 8:
            return run_mlp_trials<8, MaskDmp>(start, num_nodes, min_steps, cfg, thrash_buf);
        default:
            return run_single_chain_trials<MaskDmp>(start, num_nodes, min_steps, cfg, thrash_buf);
    }
}

double measure_working_set_ns(std::uintptr_t start,
                              std::size_t num_nodes,
                              std::size_t min_steps,
                              const Config& cfg,
                              const unsigned char* thrash_buf) {
    if (cfg.mask_dmp) {
        return dispatch_measurement<true>(start, num_nodes, min_steps, cfg, thrash_buf);
    }
    return dispatch_measurement<false>(start, num_nodes, min_steps, cfg, thrash_buf);
}

bool parse_size_arg(std::string_view sv, std::size_t& out) noexcept {
    if (sv.empty()) {
        return false;
    }
    const auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
    return ec == std::errc{} && ptr == sv.data() + sv.size() && out > 0;
}

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n\n"
        << "Range & Sampling:\n"
        << "  --min-kb <KB>          Minimum working set size in KB (default: 4)\n"
        << "  --max-kb <KB>          Maximum working set size in KB (default: 262144)\n"
        << "  --trials <N>           Trials per size, median taken (default: 3; 1 = no filter)\n\n"
        << "Mitigation Switches (toggle off to observe measurement distortions):\n"
        << "  --pattern <mode>       sattolo (default) | fisher-yates | sequential\n"
        << "  --no-random            Alias for --pattern sequential (triggers stream prefetcher)\n"
        << "  --stride <bytes>       Slot stride, power of 2 >= 8 (default: 128; try 64 or 256)\n"
        << "  --mlp <1|2|4|8>        Parallel pointer chains (default: 1; >1 exposes OoO MLP)\n"
        << "  --no-dep               Alias for --mlp 8 (breaks single-chain load serialization)\n"
        << "  --no-hugepage          Disable huge-page align & THP (exposes TLB page-walk cost)\n"
        << "  --no-unroll            Disable 64x loop unrolling (exposes loop branch overhead)\n"
        << "  --no-warmup            Evict caches & time 1 cold pass (exposes compulsory misses)\n"
        << "  --no-cpu-warmup        Skip initial CPU turbo frequency spin-up\n"
        << "  --no-pin               Disable CPU core pinning / P-core QoS boost\n"
        << "  --ecore                Schedule onto Efficiency cores (macOS QOS_CLASS_BACKGROUND)\n"
        << "  --mask-dmp             XOR-mask pointers in memory to defeat DMP (+1 cyc XOR cost)\n";
}

enum class ParseResult { kOk, kHelp, kError };

ParseResult parse_args(int argc, char* argv[], Config& cfg) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return ParseResult::kHelp;
        }
        if ((arg == "--min-kb" || arg == "--max-kb" || arg == "--trials" ||
             arg == "--stride" || arg == "--mlp") &&
            i + 1 < argc) {
            std::size_t val = 0;
            if (!parse_size_arg(argv[++i], val)) {
                std::cerr << "Invalid numeric value for " << arg << "\n";
                return ParseResult::kError;
            }
            if (arg == "--min-kb") cfg.min_kb = val;
            if (arg == "--max-kb") cfg.max_kb = val;
            if (arg == "--trials") cfg.trials = val;
            if (arg == "--stride") cfg.stride = val;
            if (arg == "--mlp") cfg.mlp_chains = val;
        } else if (arg == "--pattern" && i + 1 < argc) {
            const std::string_view mode{argv[++i]};
            if (mode == "sattolo") {
                cfg.pattern = AccessPattern::kSattolo;
            } else if (mode == "fisher-yates") {
                cfg.pattern = AccessPattern::kFisherYates;
            } else if (mode == "sequential") {
                cfg.pattern = AccessPattern::kSequential;
            } else {
                std::cerr << "Unknown --pattern mode: " << mode
                          << " (expected sattolo | fisher-yates | sequential)\n";
                return ParseResult::kError;
            }
        } else if (arg == "--no-random") {
            cfg.pattern = AccessPattern::kSequential;
        } else if (arg == "--no-dep") {
            cfg.mlp_chains = kMaxMlpChains;
        } else if (arg == "--hugepage") {
            cfg.use_hugepage = true;
        } else if (arg == "--no-hugepage") {
            cfg.use_hugepage = false;
        } else if (arg == "--unroll") {
            cfg.unroll_loop = true;
        } else if (arg == "--no-unroll") {
            cfg.unroll_loop = false;
        } else if (arg == "--warmup") {
            cfg.cache_warmup = true;
        } else if (arg == "--no-warmup") {
            cfg.cache_warmup = false;
        } else if (arg == "--cpu-warmup") {
            cfg.cpu_warmup = true;
        } else if (arg == "--no-cpu-warmup") {
            cfg.cpu_warmup = false;
        } else if (arg == "--pin") {
            cfg.thread_policy = ThreadPolicy::kPinPerf;
        } else if (arg == "--no-pin") {
            cfg.thread_policy = ThreadPolicy::kDefault;
        } else if (arg == "--ecore") {
            cfg.thread_policy = ThreadPolicy::kEfficiency;
        } else if (arg == "--mask-dmp") {
            cfg.mask_dmp = true;
        } else if (arg == "--no-mask-dmp") {
            cfg.mask_dmp = false;
        } else {
            std::cerr << "Unknown or incomplete option: " << arg << " (try --help)\n";
            return ParseResult::kError;
        }
    }

    if (cfg.min_kb > cfg.max_kb) {
        std::cerr << "--min-kb must be <= --max-kb\n";
        return ParseResult::kError;
    }
    if (cfg.stride < sizeof(std::uintptr_t) || !is_power_of_two(cfg.stride)) {
        std::cerr << "--stride must be a power of 2 and >= " << sizeof(std::uintptr_t) << "\n";
        return ParseResult::kError;
    }
    if (cfg.mlp_chains != 1 && cfg.mlp_chains != 2 &&
        cfg.mlp_chains != 4 && cfg.mlp_chains != 8) {
        std::cerr << "--mlp must be one of: 1, 2, 4, 8\n";
        return ParseResult::kError;
    }
    return ParseResult::kOk;
}

std::string format_bytes(std::size_t bytes) {
    if (bytes == 0) {
        return "N/A";
    }
    constexpr std::size_t kKB = 1024;
    constexpr std::size_t kMB = 1024 * 1024;
    constexpr std::size_t kGB = 1024ULL * 1024 * 1024;

    auto fmt_unit = [](std::size_t val, std::size_t unit, const char* suffix) {
        if (val % unit == 0) {
            return std::to_string(val / unit) + suffix;
        }
        if ((val * 100) % unit == 0) {
            const std::size_t whole = val / unit;
            const std::size_t frac = ((val % unit) * 100) / unit;
            std::string frac_str = (frac < 10 ? "0" : "") + std::to_string(frac);
            if (frac_str.back() == '0') {
                frac_str.pop_back();
            }
            return std::to_string(whole) + "." + frac_str + suffix;
        }
        return std::string{};
    };

    if (bytes >= kGB) {
        if (auto s = fmt_unit(bytes, kGB, " GB"); !s.empty()) return s;
    }
    if (bytes >= kMB) {
        if (auto s = fmt_unit(bytes, kMB, " MB"); !s.empty()) return s;
    }
    if (bytes >= kKB && bytes % kKB == 0) {
        return std::to_string(bytes / kKB) + " KB";
    }
    return std::to_string(bytes) + " B";
}

std::string format_size(std::size_t kb) {
    return format_bytes(kb * 1024);
}

struct CacheTierProfile {
    std::string label;
    std::size_t l1i_bytes = 0;
    std::size_t l1d_bytes = 0;
    std::size_t l2_bytes = 0;
    std::size_t l3_bytes = 0;
    std::size_t cpus_per_l2 = 0;
};

struct CpuCacheInfo {
    std::string cpu_brand;
    std::size_t line_bytes = 0;
    std::vector<CacheTierProfile> profiles;
};

#if defined(__APPLE__)
std::size_t sysctl_size(const char* name) noexcept {
    std::uint64_t v64 = 0;
    std::size_t len = sizeof(v64);
    if (::sysctlbyname(name, &v64, &len, nullptr, 0) != 0) {
        return 0;
    }
    if (len == sizeof(std::uint32_t)) {
        std::uint32_t v32 = 0;
        std::memcpy(&v32, &v64, sizeof(v32));
        return static_cast<std::size_t>(v32);
    }
    if (len == sizeof(std::uint64_t)) {
        return static_cast<std::size_t>(v64);
    }
    return 0;
}

std::string sysctl_string(const char* name) {
    std::size_t len = 0;
    if (::sysctlbyname(name, nullptr, &len, nullptr, 0) != 0 || len == 0) {
        return {};
    }
    std::string s(len, '\0');
    if (::sysctlbyname(name, s.data(), &len, nullptr, 0) != 0) {
        return {};
    }
    while (!s.empty() && (s.back() == '\0' || s.back() == '\n' || s.back() == ' ')) {
        s.pop_back();
    }
    return s;
}
#elif defined(__linux__)
std::string trim_sv(std::string_view sv) {
    while (!sv.empty() &&
           (sv.front() == ' ' || sv.front() == '\t' || sv.front() == '\r' || sv.front() == '\n')) {
        sv.remove_prefix(1);
    }
    while (!sv.empty() &&
           (sv.back() == ' ' || sv.back() == '\t' || sv.back() == '\r' || sv.back() == '\n')) {
        sv.remove_suffix(1);
    }
    return std::string{sv};
}

std::string read_first_line(const std::string& path) {
    std::ifstream ifs(path);
    std::string line;
    if (std::getline(ifs, line)) {
        return trim_sv(line);
    }
    return {};
}

std::size_t parse_scaled_bytes(std::string_view sv) noexcept {
    while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t')) {
        sv.remove_prefix(1);
    }
    std::size_t val = 0;
    const auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), val);
    if (ec != std::errc{} || val == 0) {
        return 0;
    }
    const std::string_view unit{ptr, static_cast<std::size_t>(sv.data() + sv.size() - ptr)};
    if (unit.empty() || unit == "B") return val;
    if (unit == "K" || unit == "KB") return val * 1024;
    if (unit == "M" || unit == "MB") return val * 1024 * 1024;
    if (unit == "G" || unit == "GB") return val * 1024ULL * 1024 * 1024;
    return val;
}
#endif

CpuCacheInfo detect_cpu_cache_info([[maybe_unused]] int pinned_cpu,
                                   [[maybe_unused]] ThreadPolicy policy) {
    CpuCacheInfo info{};
#if defined(__APPLE__)
    info.cpu_brand = sysctl_string("machdep.cpu.brand_string");
    info.line_bytes = sysctl_size("hw.cachelinesize");

    const std::size_t nlevels = sysctl_size("hw.nperflevels");
    for (std::size_t lvl = 0; lvl < nlevels; ++lvl) {
        const std::string prefix = "hw.perflevel" + std::to_string(lvl);
        CacheTierProfile prof{};
        prof.label = sysctl_string((prefix + ".name").c_str());
        if (prof.label.empty()) {
            prof.label = "Level " + std::to_string(lvl);
        }
        if (nlevels > 1) {
            if ((policy == ThreadPolicy::kPinPerf && lvl == 0) ||
                (policy == ThreadPolicy::kEfficiency && lvl + 1 == nlevels)) {
                prof.label += "*";
            }
        }
        prof.l1i_bytes = sysctl_size((prefix + ".l1icachesize").c_str());
        prof.l1d_bytes = sysctl_size((prefix + ".l1dcachesize").c_str());
        prof.l2_bytes = sysctl_size((prefix + ".l2cachesize").c_str());
        prof.l3_bytes = sysctl_size((prefix + ".l3cachesize").c_str());
        if (prof.l3_bytes == 0 && nlevels == 1) {
            prof.l3_bytes = sysctl_size("hw.l3cachesize");
        }
        prof.cpus_per_l2 = sysctl_size((prefix + ".cpusperl2").c_str());

        if (prof.l1d_bytes > 0 || prof.l2_bytes > 0) {
            info.profiles.push_back(std::move(prof));
        }
    }

    if (info.profiles.empty()) {
        CacheTierProfile prof{};
        prof.label = "CPU";
        prof.l1i_bytes = sysctl_size("hw.l1icachesize");
        prof.l1d_bytes = sysctl_size("hw.l1dcachesize");
        prof.l2_bytes = sysctl_size("hw.l2cachesize");
        prof.l3_bytes = sysctl_size("hw.l3cachesize");
        if (prof.l1d_bytes > 0 || prof.l2_bytes > 0) {
            info.profiles.push_back(std::move(prof));
        }
    }
#elif defined(__linux__)
    {
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuinfo, line)) {
            const auto pos = line.find(':');
            if (pos == std::string::npos) continue;
            const std::string key = trim_sv(std::string_view{line.data(), pos});
            if (key == "model name" || key == "Model" || key == "Hardware") {
                info.cpu_brand = trim_sv(std::string_view{line.data() + pos + 1});
                if (!info.cpu_brand.empty()) break;
            }
        }
    }

    const int target_cpu = pinned_cpu >= 0 ? pinned_cpu : 0;
    CacheTierProfile prof{};
    prof.label = "CPU " + std::to_string(target_cpu) + (pinned_cpu >= 0 ? "*" : "");

    const std::string cache_root =
        "/sys/devices/system/cpu/cpu" + std::to_string(target_cpu) + "/cache";
    for (int idx = 0; idx < 16; ++idx) {
        const std::string dir = cache_root + "/index" + std::to_string(idx);
        const std::string level_str = read_first_line(dir + "/level");
        if (level_str.empty()) break;

        const std::size_t level = parse_scaled_bytes(level_str);
        const std::string type = read_first_line(dir + "/type");
        const std::size_t sz = parse_scaled_bytes(read_first_line(dir + "/size"));
        const std::size_t line_sz =
            parse_scaled_bytes(read_first_line(dir + "/coherency_line_size"));

        if (line_sz > 0 && info.line_bytes == 0) {
            info.line_bytes = line_sz;
        }
        if (level == 1 && type == "Instruction") {
            prof.l1i_bytes = sz;
        } else if (level == 1 && (type == "Data" || type == "Unified")) {
            prof.l1d_bytes = sz;
        } else if (level == 2) {
            prof.l2_bytes = sz;
        } else if (level == 3) {
            prof.l3_bytes = sz;
        }
    }

    if (prof.l1d_bytes == 0 && prof.l2_bytes == 0) {
#ifdef _SC_LEVEL1_ICACHE_SIZE
        const long l1i = ::sysconf(_SC_LEVEL1_ICACHE_SIZE);
        if (l1i > 0) prof.l1i_bytes = static_cast<std::size_t>(l1i);
#endif
#ifdef _SC_LEVEL1_DCACHE_SIZE
        const long l1d = ::sysconf(_SC_LEVEL1_DCACHE_SIZE);
        if (l1d > 0) prof.l1d_bytes = static_cast<std::size_t>(l1d);
#endif
#ifdef _SC_LEVEL1_DCACHE_LINESIZE
        const long cl = ::sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
        if (cl > 0 && info.line_bytes == 0) info.line_bytes = static_cast<std::size_t>(cl);
#endif
#ifdef _SC_LEVEL2_CACHE_SIZE
        const long l2 = ::sysconf(_SC_LEVEL2_CACHE_SIZE);
        if (l2 > 0) prof.l2_bytes = static_cast<std::size_t>(l2);
#endif
#ifdef _SC_LEVEL3_CACHE_SIZE
        const long l3 = ::sysconf(_SC_LEVEL3_CACHE_SIZE);
        if (l3 > 0) prof.l3_bytes = static_cast<std::size_t>(l3);
#endif
    }

    if (prof.l1d_bytes > 0 || prof.l2_bytes > 0) {
        info.profiles.push_back(std::move(prof));
    }
#endif
    return info;
}

const char* pattern_name(AccessPattern pat) noexcept {
    switch (pat) {
        case AccessPattern::kSattolo:
            return "sattolo (single-cycle random)";
        case AccessPattern::kFisherYates:
            return "fisher-yates [WARN: multi-cycle trap]";
        case AccessPattern::kSequential:
            return "sequential [OFF: triggers stream prefetcher]";
    }
    return "unknown";
}

const char* thread_policy_name(ThreadPolicy pol) noexcept {
    switch (pol) {
        case ThreadPolicy::kPinPerf:
            return "pin-perf (ON)";
        case ThreadPolicy::kDefault:
            return "os-default (OFF)";
        case ThreadPolicy::kEfficiency:
            return "efficiency-core (E-Core)";
    }
    return "unknown";
}

void print_system_summary(const CpuCacheInfo& cpu_info,
                          const PageConfig& page_cfg,
                          const Config& cfg) {
    constexpr int kLabelWidth = 21;
    std::cout << "======================================================================\n";
    if (!cpu_info.cpu_brand.empty()) {
        std::cout << std::left << std::setw(kLabelWidth) << "CPU"
                  << ": " << cpu_info.cpu_brand << '\n';
    }
    std::cout << std::left << std::setw(kLabelWidth) << "Cache Line" << ": "
              << format_bytes(cpu_info.line_bytes)
              << " (Slot Stride = " << cfg.stride << " B)\n";

    for (const auto& prof : cpu_info.profiles) {
        const std::string tag = "Cache (" + prof.label + ")";
        std::cout << std::left << std::setw(kLabelWidth) << tag << ": "
                  << "L1I = " << format_bytes(prof.l1i_bytes)
                  << " | L1D = " << format_bytes(prof.l1d_bytes)
                  << " | L2 = " << format_bytes(prof.l2_bytes);
        if (prof.cpus_per_l2 > 1) {
            std::cout << " (" << prof.cpus_per_l2 << " cores/L2)";
        }
        std::cout << " | L3 = " << format_bytes(prof.l3_bytes) << '\n';
    }

    std::cout << std::left << std::setw(kLabelWidth) << "Memory Pages" << ": "
              << "Base Page = " << format_bytes(page_cfg.base_page_bytes);
    if (page_cfg.huge_page_bytes > 0) {
        std::cout << " | Huge Page (THP) = " << format_bytes(page_cfg.huge_page_bytes)
                  << (cfg.use_hugepage ? " [ON]" : " [OFF]");
    } else if (!cfg.use_hugepage) {
        std::cout << " | Huge Page = [OFF]";
    }
    std::cout << '\n';

    std::cout << std::left << std::setw(kLabelWidth) << "Pattern & Chains" << ": "
              << pattern_name(cfg.pattern) << " | MLP Chains = " << cfg.mlp_chains
              << (cfg.mlp_chains > 1 ? " [OFF: parallel loads]" : " (serial)") << '\n';

    std::cout << std::left << std::setw(kLabelWidth) << "Controls" << ": "
              << "Unroll = " << (cfg.unroll_loop ? "64x" : "OFF (1x)")
              << " | Warmup = " << (cfg.cache_warmup ? "ON" : "OFF (cold 1-pass)")
              << " | CPU-Spin = " << (cfg.cpu_warmup ? "ON" : "OFF") << '\n';

    std::cout << std::left << std::setw(kLabelWidth) << "Scheduling & DMP" << ": "
              << "Thread = " << thread_policy_name(cfg.thread_policy)
              << " | DMP-Mask = " << (cfg.mask_dmp ? "ON (XOR)" : "OFF (raw ptr)")
              << " | Trials = " << cfg.trials
              << "\n======================================================================\n";
}

std::string make_bar(double latency_ns) {
    constexpr double kScaleNsPerCol = 3.0;
    constexpr std::size_t kMaxCols = 36;
    const auto cols = std::min<std::size_t>(
        kMaxCols,
        static_cast<std::size_t>(latency_ns / kScaleNsPerCol + 0.5));
    return std::string(cols == 0 ? 1 : cols, '#');
}

}  // namespace

int main(int argc, char* argv[]) {
    Config cfg{};
    const ParseResult pr = parse_args(argc, argv, cfg);
    if (pr == ParseResult::kHelp) {
        return 0;
    }
    if (pr == ParseResult::kError) {
        return 1;
    }

    const int pinned_cpu = configure_thread_execution(cfg.thread_policy);
    if (cfg.cpu_warmup) {
        warmup_cpu_frequency();
    }

    const PageConfig page_cfg = detect_page_config();
    const CpuCacheInfo cpu_info = detect_cpu_cache_info(pinned_cpu, cfg.thread_policy);

    AlignedByteBuffer thrash_buf;
    if (!cfg.cache_warmup) {
        thrash_buf = allocate_buffer(kColdEvictBytes, 64, page_cfg, false);
        std::memset(thrash_buf.get(), 0x5a, kColdEvictBytes);
    }

    print_system_summary(cpu_info, page_cfg, cfg);

    std::cout << std::left << std::setw(14) << "Working Set" << " | "
              << std::right << std::setw(12) << "Latency (ns)" << " | "
              << "Visual Scale (~3 ns / '#')\n"
              << "---------------+--------------+--------------------------------------\n";

    for (std::size_t kb = cfg.min_kb; kb <= cfg.max_kb; kb *= 2) {
        const std::size_t bytes = kb * 1024;
        const std::size_t num_nodes = std::max<std::size_t>(1, bytes / cfg.stride);
        const std::size_t alloc_bytes = num_nodes * cfg.stride;

        auto buf = allocate_buffer(alloc_bytes, cfg.stride, page_cfg, cfg.use_hugepage);
        build_pointer_chain(buf.get(), num_nodes, cfg.stride, cfg.pattern, cfg.mask_dmp);

        const std::size_t min_steps =
            (kb <= 512) ? (1ULL << 24) : (kb <= 16'384 ? (1ULL << 22) : (1ULL << 21));

        const double latency_ns = measure_working_set_ns(
            slot_addr(buf.get(), 0, cfg.stride), num_nodes, min_steps, cfg, thrash_buf.get());

        std::cout << std::right << std::setw(12) << format_size(kb) << "   | "
                  << std::fixed << std::setprecision(2) << std::setw(9)
                  << latency_ns << " ns | " << make_bar(latency_ns) << '\n'
                  << std::flush;

        if (kb > cfg.max_kb / 2) {
            break;
        }
    }

    return 0;
}
