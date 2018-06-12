
using System;
using System.Runtime.InteropServices;

class OpenVrMarshal
{
    public static T[] FromIntPtrToArray<T>(IntPtr ptr, uint length)
    {
        var array = new T[length];
        var object_size = Marshal.SizeOf(typeof(T));

        for (int i = 0; i < length; i++)
        {
            IntPtr ptr_offset = new IntPtr(ptr.ToInt64() + i * object_size);
            array[i] = (T)Marshal.PtrToStructure(ptr_offset, typeof(T));
        }

        return array;
    }
}
