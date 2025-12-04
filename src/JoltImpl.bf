#define STANDALONE_IMPL
//#define RAYLIB_IMPL

#if STANDALONE_IMPL
namespace Jolt;
using System;
using System.Interop;

[CRepr] struct JPH_Vec3
{
	public float x;
	public float y;
	public float z;

	public this(float x, float y, float z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
}
#elif RAYLIB_IMPL
namespace Jolt;
typealias JPH_Vec3 = RaylibBeef.Vector3;
#endif