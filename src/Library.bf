namespace Jolt;

static class Jolt
{
	protected const let dll = "joltc" +
#if BF_PLATFORM_WINDOWS
	".dll";
#elif BF_PLATFORM_MACOS
	".dylib";
#else
	".so";
#endif
}