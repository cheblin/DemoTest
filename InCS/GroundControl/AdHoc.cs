
// AdHoc protocol - data interchange format and source code generator
// Copyright 2019 Chikirev Sirguy, Unirail Group. All rights reserved.
// info@unirail.org
// https://github.com/cheblin/AdHoc-protocol
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
using System;
using System.Diagnostics;


namespace org.unirail
{
    public static partial class AdHoc
    {
        private static readonly byte[] Ol = {0, 1, 3, 7, 15, 31, 63, 127, 255};
        private static readonly byte[] lO = {0, 128, 192, 224, 240, 248, 252, 254, 255};

        public static ulong get_bytes(byte[] src, int BYTE, int bytes)
        {
            int hi = 0, lo = 0;
            switch(bytes)
            {
                case 8:
                    hi |= src[BYTE + 7] << 24;
                goto case 7;
                case 7:
                    hi |= src[BYTE + 6] << 16;
                goto case 6;
                case 6:
                    hi |= src[BYTE + 5] << 8;
                goto case 5;
                case 5:
                    hi |= src[BYTE + 4];
                goto case 4;
                case 4:
                    lo |= src[BYTE + 3] << 24;
                goto case 3;
                case 3:
                    lo |= src[BYTE + 2] << 16;
                goto case 2;
                case 2:
                    lo |= src[BYTE + 1] << 8;
                goto case 1;
                case 1:
                    lo |= src[BYTE] & 0xFF;
                    break;
            }
            return ((ulong)(uint) hi << 32) | (uint) lo;
        }

        public static uint set_bytes(ulong src, int bytes, byte[] dst, int BYTE)
        {
            uint hi = (uint)(src >> 32), lo = (uint) src;
            switch(bytes)
            {
                case 8:
                    dst[BYTE + 7] = (byte)(hi >> 24);
                goto case 7;
                case 7:
                    dst[BYTE + 6] = (byte)(hi >> 16);
                goto case 6;
                case 6:
                    dst[BYTE + 5] = (byte)(hi >> 8);
                goto case 5;
                case 5:
                    dst[BYTE + 4] = (byte) hi;
                goto case 4;
                case 4:
                    dst[BYTE + 3] = (byte)(lo >> 24);
                goto case 3;
                case 3:
                    dst[BYTE + 2] = (byte)(lo >> 16);
                goto case 2;
                case 2:
                    dst[BYTE + 1] = (byte)(lo >> 8);
                goto case 1;
                case 1:
                    dst[BYTE] = (byte) lo;
                    break;
            }
            return (uint)(BYTE + bytes);
        }

        public static ulong get_bits(byte[] src, int bit, int bits)
        {
            var src_byte = bit >> 3;
            bit &= 7;
            if(64              < bits) bits = 64;
            else if(bit + bits < 9) return (ulong)((src[src_byte] >> bit) & Ol[bits]);
            ulong dst = 0;
            for(int i = 0, last = ((bit + bits) >> 3) << 3; i < last; i += 8)
                dst |= (src[src_byte++] & 0xFFUL) << i;
            dst >>= bit;
            bit = (bit + bits) & 7;
            if(0 < bit) dst |= (ulong)(src[src_byte] & Ol[bit]) << (bits - bit);
            return dst;
        }

        public static void set_0(byte[] dst, int bit, int bits)
        {
            var dst_byte = bit >> 3;
            bit &= 7;
            if(8 < bit + bits)
            {
                if(0 < bit)
                {
                    dst[dst_byte] &= Ol[bit];
                    if((bits -= 8 - bit) == 0) return;
                    dst_byte++;
                }
                if(0            < (bits & 7)) dst[dst_byte + (bits >> 3)] &= lO[8 - (bits & 7)];
                if((bits >>= 3) == 0) return;
                for(var i = dst_byte + bits; dst_byte <= --i;)
                    dst[i] = 0;
            }
            else { dst[dst_byte] &= (byte)(Ol[bit] | lO[8 - (bit + bits)]); }
        }

        public static void set_bits(ulong src, int bits, byte[] dst, int bit)
        {
            var usrc     = src;
            var dst_byte = bit >> 3;
            bit &= 7;
            if(8 < bit + bits)
            {
                if(0 < bit)
                {
                    dst[dst_byte] = (byte)(dst[dst_byte] & Ol[bit] |
                                           ((int) usrc & Ol[
                                                8 - bit
                                            ]) << bit);
                    dst_byte++;
                    usrc >>= 8 - bit;
                    bits -=  8 - bit;
                }
                for(int BYTE = 0, bytes = bits >> 3; BYTE < bytes; BYTE++, usrc >>= 8)
                    dst[dst_byte++] = (byte)(usrc & 0xFF);
                if(0 < (bits &= 7))
                    dst[dst_byte] = (byte)(dst[dst_byte] & lO[8 - bits]
                                           | (int) usrc  & Ol[bits]);
            }
            else { dst[dst_byte] = (byte)((dst[dst_byte] & (Ol[bit] | lO[8 - bit - bits])) | (((int) usrc & Ol[bits]) << bit)); }
        }

        public static void copy_bits(byte[] src, int src_bit, int bits, byte[] dst, int dst_bit)
        {
            if(bits == 0 || src == dst && src_bit == dst_bit) return;
            var count = bits >> 6;
            bits &= 0x3F;
            if(src == dst && src_bit < dst_bit)
            {
                src_bit += count * 64;
                dst_bit += count * 64;
                if(0    < bits) set_bits(get_bits(src,                                  src_bit, bits), bits, dst, dst_bit);
                for(; 0 < count--; src_bit -= 64, dst_bit -= 64, set_bits(get_bits(src, src_bit, 64),   64,   dst, dst_bit)) ;
            }
            else
            {
                for(; 0 < count; set_bits(get_bits(src, src_bit, 64),   64,   dst, dst_bit), src_bit += 64, dst_bit += 64, count--) ;
                if(0    < bits) set_bits(get_bits(src,  src_bit, bits), bits, dst, dst_bit);
            }
        }

        public static int first_1(byte[] bytes, int bit, int bits, bool existence)
        {
            if(bits < 1) return -1;
            var _1BYTE = bit >> 3;
            var v      = bytes[_1BYTE];
            bit &= 7;
            if(bits == 1) return (v & (1 << bit)) == 0 ? -1 : 0;
            var add = 0;
            {
                if(0 < bit)
                {
                    if(0 < (v >>= bit))
                    {
                        if(bit + bits < 8 && (v & Ol[bits]) == 0)
                            return -1;
                        goto sBreak;
                    }
                    if(bit + bits < 8) return -1;
                    bits -= add = 8 - bit;
                    _1BYTE++;
                }
                else
                {
                    if(bits < 9)
                        if(v == 0 || (v & Ol[bits]) == 0)
                            return -1;
                        else
                            goto sBreak;
                }
                var last = _1BYTE + (bits >> 3);
                for(var BYTE = _1BYTE; BYTE < last; BYTE++)
                    if(0 < (v = bytes[BYTE]))
                    {
                        add += (BYTE - _1BYTE) << 3;
                        goto sBreak;
                    }
                if((bits &= 7) == 0 || (v = (byte)(bytes[last] & Ol[bits])) == 0) return -1;
                add += (last - _1BYTE) << 3;
            }
            sBreak:
            if(existence) return int.MaxValue;
            for(var i = 0;; i++)
                if(((v >> i) & 1) == 1)
                    return add + i;
        }


        public static int bits2bytes(int bits) { return bits < 1 ? 0 : 1 + ((bits - 1) >> 3); }


        public class Pack
        {
            public delegate void Handler<Channel, Pack>(Channel src, Pack pack);

            public          byte[] bytes;
            public readonly Meta   meta;

            public Pack(Meta meta) { this.meta = meta; }


            public class Meta
            {
                internal readonly int _2;
                internal readonly int _4;
                internal readonly int _8;
                public readonly   int packMinBytes;
                internal readonly ushort BITS_lenINbytes_bits;
                public readonly ushort nesting_max;

                internal readonly int     field_0_bit;
                public readonly   Field[] fields_reqs;
                public readonly   Field[] fields_opts;


                public readonly int id;


                public Meta(int id) : this(id, 0, 0, 0, 0, 1, 0, 0, 0, 0) { }

                public Meta(int id, int _2, int _4, int _8, int packMinBytes, int nesting_max, int field_0_bit) : this(id, _2, _4, _8, packMinBytes, nesting_max, field_0_bit, 0, 0, 0) { }

                public Meta(int id, int _2, int _4, int _8, int packMinBytes, int nesting_max, int field_0_bit, int BITS_lenINbytes_bits, int fields_reqs, int fields_opts)
                {
                    this.id                   = id;
                    this._2                   = _2;
                    this._4                   = _4;
                    this._8                   = _8;
                    this.packMinBytes         = packMinBytes;
                    this.BITS_lenINbytes_bits = (ushort) BITS_lenINbytes_bits;
                    this.nesting_max          = (ushort) nesting_max;
                    this.field_0_bit          = field_0_bit;
                    this.fields_opts          = 0 < fields_opts ? new Field[fields_opts] : null;
                    this.fields_reqs          = 0 < fields_reqs ? new Field[fields_reqs] : null;
                }


                public class Field
                {





                    internal readonly int const_dims_total;

                    public readonly Meta datatype;

                    public readonly int[] var_dims;


                    internal readonly ushort field_info_bits;



                    internal readonly byte sparse_bits;
                    internal readonly int length;

                    internal readonly sbyte size;

                    internal readonly byte type;

                    internal readonly bool varint;

                    public Field(int type, bool varint, int length, int size, int const_dims_total, int field_info_bits, int sparse_bits, Meta datatype, params int[] var_dims)
                    {
                        this.type             = (byte) type;
                        this.varint           = varint;
                        this.length           = length;
                        this.size             = (sbyte) size;
                        this.const_dims_total = const_dims_total;
                        this.field_info_bits  = (ushort) field_info_bits;
                        this.sparse_bits      = (byte) sparse_bits;
                        this.datatype         = datatype;
                        this.var_dims         = var_dims != null && 0 < var_dims.Length ? var_dims : null;
                    }


                    public class CursorBase
                    {
                        internal CursorBase next_;

                        internal CursorBase prev;

                        public CursorBase() { }

                        internal CursorBase(CursorBase prev)
                        {
                            if((this.prev = prev) != null) prev.next_ = this;
                        }

                        public int    origin;
                        public byte[] bytes;
                        public Meta   meta;


                        internal int BIT_S = -1;
                        internal int BIT_E = -1;


                        internal int BYTE_E = -1;
                        internal int BYTE_S = -1;

                        public int field_bit;

                        internal int item_len;

                        public virtual void wrap(Meta meta)
                        {
                            this.meta = meta;
                            bytes     = new byte[meta.packMinBytes];
                            reset();
                        }

                        public virtual Pack unwrap()
                        {
                            if(meta == null) return null;
                            var dst = new Pack(meta) {bytes = bytes};
                            meta  = null;
                            bytes = null;
                            return dst;
                        }
                        public virtual void wrap(Pack src)
                        {
                            origin    = 0;
                            bytes     = src.bytes;
                            meta      = src.meta;
                            src.bytes = null;
                            reset();
                        }

                        public bool equal(CursorBase bytes, int len)
                        {
                            if(len == 0) return false;
                            for(; 0 < --len;)
                                if(this.bytes[origin + len] != bytes.bytes[bytes.origin + len])
                                    return false;
                            return true;
                        }





                        internal int next_field_bit()
                        {
                            if(field_bit - meta.field_0_bit < meta.fields_opts.Length - 1)
                            {
                                var bit = origin * 8 + (field_bit < 0 ? meta.field_0_bit : field_bit + 1);
                                var i   = first_1(bytes, bit, Math.Min(BIT_E - bit, meta.fields_opts.Length), false);
                                return i < 0 ? -1 : i + bit - (origin * 8);
                            }
                            return -1;
                        }

                        public Field getField() { return meta.fields_opts[field_bit - meta.field_0_bit]; }

                        protected internal virtual bool reset()
                        {
                            field_bit = -1;
                            var len_bits      = meta.BITS_lenINbytes_bits;
                            BYTE_E   = BYTE_S = origin + meta.packMinBytes + (len_bits == 0 ? 0 : (int) get_bits(bytes, origin * 8 + meta.field_0_bit - len_bits, len_bits));
                            BIT_E    = BIT_S  = BYTE_E << 3;
                            item_len = 0;
                            return true;
                        }



                        internal int type_len(Meta META, int ORIGIN)
                        {
                            if(META.fields_opts == null) return META.packMinBytes;
                            var bit_0    = ORIGIN * 8 + META.field_0_bit;
                            var len_bits = META.BITS_lenINbytes_bits;
                            var LAST_BYTE = ORIGIN + META.packMinBytes + (len_bits == 0 ? 0 : (int) get_bits(bytes, bit_0 - len_bits, len_bits));
                            var fb = first_1(bytes, bit_0, Math.Min(LAST_BYTE * 8 - bit_0, META.fields_opts.Length), false);
                            if(fb == -1) return LAST_BYTE - ORIGIN;
                            fb += META.field_0_bit;
                            int
                            _BIT_E     = BIT_E,
                            _BIT_S     = BIT_S,
                            _BYTE_S    = BYTE_S,
                            _BYTE_E    = BYTE_E,
                            _item_type = item_len,
                            _origin    = origin,
                            _field_bit = field_bit;
                            var _meta = meta;
                            meta      = META;
                            field_bit = fb;
                            origin    = ORIGIN;
                            BYTE_E    = LAST_BYTE;
                            BIT_E     = LAST_BYTE << 3;
                            do
                                set_E();
                            while(-1 < (field_bit = next_field_bit()));
                            var ret = BYTE_E - ORIGIN;
                            BIT_E     = _BIT_E;
                            BIT_S     = _BIT_S;
                            BYTE_S    = _BYTE_S;
                            BYTE_E    = _BYTE_E;
                            item_len  = _item_type;
                            origin    = _origin;
                            field_bit = _field_bit;
                            meta      = _meta;
                            return ret;
                        }





                        internal void set_E()
                        {
                            var fld = getField();
                            BYTE_S = BYTE_E;
                            var bit = BIT_S = BIT_E;
                            var items = fld.const_dims_total;
                            if(fld.var_dims != null)
                                for(var i = 0; i < fld.var_dims.Length; i++)
                                    items *= (int) get_bits(bytes, bit -= fld.var_dims[i], fld.var_dims[i]);
                            BIT_E -= fld.field_info_bits;
                            var len = 0;
                            switch(fld.type)
                            {
                                case 1:
                                    BYTE_E += items * (fld.datatype == null ? fld.length * fld.size : type_len(fld.datatype, BYTE_S));
                                    break;
                                case 3:
                                    items  *= (int) get_bits(bytes, BIT_E, fld.length);
                                    BYTE_E += items * fld.size;
                                    break;
                                case 5:
                                    while(0 < items--)
                                        len += (int) get_bits(bytes, BIT_E -= fld.length, fld.length);
                                    BYTE_E += len * fld.size;
                                    break;
                                case 7:
                                    BIT_E -= items * fld.length * fld.size;
                                    break;
                                case 9:
                                    items *= (int) get_bits(bytes, BIT_E, fld.length);
                                    BIT_E -= items * fld.size;
                                    break;
                                case 11:
                                    while(0 < items--)
                                    {
                                        var bits = (int) get_bits(bytes, BIT_E -= fld.length, fld.length) * fld.size;
                                        BIT_E -= bits;
                                    }
                                    break;
                                default:
                                    if((items = (int) get_bits(bytes, BIT_E, fld.sparse_bits)) == 0) return;
                                    bit = BIT_E;
                                    switch(fld.type)
                                    {
                                        case 2:
                                            BIT_E -= items;
                                            if(fld.datatype == null)
                                            {
                                                while(BIT_E < bit--)
                                                    if((bytes[bit >> 3] & (1 << (bit & 7))) == 0)
                                                        items--;
                                                BYTE_E += fld.length * items * fld.size;
                                            }
                                            else
                                                while(BIT_E < bit--)
                                                    if((bytes[bit >> 3] & (1 << (bit & 7))) != 0)
                                                        BYTE_E += type_len(fld.datatype, BYTE_E);
                                            break;
                                        case 4:
                                            BIT_E -= items;
                                            len   = (int) get_bits(bytes, bit + 2 * fld.sparse_bits, fld.length);
                                            while(BIT_E < bit--)
                                                if((bytes[bit >> 3] & (1 << (bit & 7))) == 0)
                                                    items--;
                                            BYTE_E += len * items * fld.size;
                                            break;
                                        case 6:
                                            while(0 < items--)
                                                if((bytes[--bit >> 3] & (1 << (bit & 7))) != 0)
                                                    len += (int) get_bits(bytes, bit -= fld.length, fld.length);
                                            BIT_E  =  bit;
                                            BYTE_E += len * fld.size;
                                            break;
                                        case 8:
                                            for(var bits = fld.length * fld.size; 0 < items--;)
                                                if((bytes[--bit >> 3] & (1 << (bit & 7))) != 0)
                                                    bit -= bits;
                                            BIT_E = bit;
                                            break;
                                        case 10:
                                            for(var bits = (int) get_bits(bytes, bit + 2 * fld.sparse_bits, fld.length) * fld.size; 0 < items--;)
                                                if((bytes[--bit >> 3] & (1 << (bit & 7))) != 0)
                                                    bit -= bits;
                                            BIT_E = bit;
                                            break;
                                        case 12:
                                            while(0 < items--)
                                                if((bytes[--bit >> 3] & (1 << (bit & 7))) != 0)
                                                {
                                                    len = (int) get_bits(bytes, bit -= fld.length, fld.length);
                                                    bit -= len * fld.size;
                                                }
                                            BIT_E = bit;
                                            break;
                                        default:
                                            Debug.Assert(false);
                                            return;
                                    }
                                    return;
                            }
                        }
                    }
                }
            }
            public class Cursor : Meta.Field.CursorBase
            {
                public int[] var_dims = null;
                public int   BIT      = -1;

                public int BYTE;

                private int field_item;

                internal int field_item_0;
                internal int field_items;

                internal int field_items_total;





                internal           int shift;
                internal           int pack_LAST_BIT;
                protected internal int pack_LAST_BYTE;
                internal           int pack_LAST_field_bit;

                internal readonly ushort depth;



                internal Cursor(Cursor prev)
                {
                    if((this.prev = prev) == null) depth = 1;
                    else
                    {
                        depth      = (ushort)(prev.depth + 1);
                        prev.next_ = this;
                    }
                }
                protected internal int set_pack_LASTS()
                {
                    var fb = next_field_bit();
                    if(fb < 0)
                    {
                        pack_LAST_BYTE      = BYTE_E;
                        pack_LAST_BIT       = BIT_E;
                        pack_LAST_field_bit = field_bit;
                        return pack_LAST_BYTE;
                    }
                    int
                    BIT_E_FX     = BIT_E,
                    BIT_S_FX     = BIT_S,
                    BYTE_S_FX    = BYTE_S,
                    BYTE_E_FX    = BYTE_E,
                    item_type_FX = item_len,
                    field_bit_FX = field_bit;
                    field_bit = fb;
                    do
                        set_E();
                    while(-1 < (field_bit = next_field_bit()));
                    pack_LAST_BYTE      = BYTE_E;
                    pack_LAST_BIT       = BIT_E;
                    pack_LAST_field_bit = field_bit;
                    BIT_E               = BIT_E_FX;
                    BIT_S               = BIT_S_FX;
                    BYTE_S              = BYTE_S_FX;
                    BYTE_E              = BYTE_E_FX;
                    item_len            = item_type_FX;
                    field_bit           = field_bit_FX;
                    return pack_LAST_BYTE;
                }

                public int length() { return (-1 < pack_LAST_BYTE ? pack_LAST_BYTE : set_pack_LASTS()) - origin; }

                public Cursor next(int origin, Meta meta)
                {
                    if(next_ == null) next_ = new Cursor(this);
                    next_.meta   = meta;
                    next_.origin = origin;
                    next_.bytes  = bytes;
                    next_.reset();
                    return (Cursor) next_;
                }

                protected internal override bool reset()
                {
                    base.reset();
                    if(meta.fields_opts == null ||
                            field_bit == -1 && next_field_bit() < 0)
                    {
                        pack_LAST_BYTE      = BYTE_E;
                        pack_LAST_BIT       = BIT_E;
                        pack_LAST_field_bit = meta.field_0_bit;
                    }
                    else pack_LAST_BYTE = pack_LAST_BIT = pack_LAST_field_bit = -1;
                    BYTE         = BYTE_S;
                    BIT          = BIT_S;
                    item_len     = 0;
                    field_item_0 = -1;
                    field_item   = int.MaxValue;
                    field_items  = 0;
                    return true;
                }

                public override Pack unwrap()
                {
                    var ret = base.unwrap();
                    for(var cur = this; (cur = (Cursor) cur.next_) != null; cur.bytes = null, cur.meta = null) ;
                    return ret;
                }

                public override void wrap(Pack src)
                {
                    origin = 0;
                    bytes  = src.bytes;
                    meta   = src.meta;
                    reset();
                }
                public bool set_field(int fbit, int each_item_size, params int[] var_dims)
                {
                    if(field_bit == fbit) return true;
                    if(each_item_size < 0 &&
                            (BIT_E                                           <= origin * 8 + fbit             ||
                             meta.fields_opts.Length                         <= fbit       - meta.field_0_bit ||
                             (bytes[origin + (fbit >> 3)] & 1 << (fbit & 7)) == 0)) return false;
                    Meta.Field fld;
                    if(fbit < field_bit) reset();
                    for(;;)
                    {
                        var fb = next_field_bit();
                        if(fbit < fb ||
                                fb   == -1)
                        {
                            if(each_item_size < 0) return false;
                            fld = meta.fields_opts[fbit - meta.field_0_bit];
                            var UDT = fld.datatype != null;
                            field_items_total = fld.const_dims_total;
                            if(fld.var_dims != null)
                                if(this.var_dims == null || this.var_dims.Length < var_dims.Length)
                                    foreach(int i in this.var_dims = var_dims)
                                        field_items_total *= i;
                                else
                                    for(var i = 0; i < fld.var_dims.Length; i++)
                                        field_items_total *= this.var_dims[i] = var_dims[i];
                            switch(fld.type)
                            {
                                case 1:
                                    insert(fbit, fld.field_info_bits, field_items_total * (UDT ? Math.Max(each_item_size, fld.datatype.packMinBytes) : fld.length * fld.size));
                                    break;
                                case 2:
                                case 6:
                                case 8:
                                case 12:
                                    insert(fbit, fld.field_info_bits, 0);
                                    break;
                                case 3:
                                    insert(fbit, fld.field_info_bits, field_items_total * each_item_size * fld.size);
                                    set_bits((ulong) each_item_size, fld.length, bytes, BIT_S - fld.field_info_bits);
                                    break;
                                case 9:
                                    insert(fbit, fld.field_info_bits                          + field_items_total * each_item_size * fld.size, 0);
                                    set_bits((ulong) each_item_size, fld.length, bytes, BIT_S - fld.field_info_bits);
                                    break;
                                case 4:
                                case 10:
                                    insert(fbit, fld.field_info_bits, 0);
                                    set_bits((ulong) each_item_size, fld.length, bytes, BIT_S - fld.field_info_bits + 2 * fld.sparse_bits);
                                    break;
                                case 5:
                                case 11:
                                    insert(fbit, fld.field_info_bits + field_items_total * fld.length, 0);
                                    break;
                                case 7:
                                    insert(fbit, fld.field_info_bits + field_items_total * fld.length * fld.size, 0);
                                    break;
                                default:
                                    Debug.Assert(false);
                                    break;
                            }
                            if(fld.var_dims != null)
                                for(int i = 0, bit = BIT_S; i < var_dims.Length; i++)
                                    set_bits((ulong) var_dims[i], fld.var_dims[i], bytes, bit -= fld.var_dims[i]);
                            break;
                        }
                        field_bit = fb;
                        set_E();
                        if(field_bit < fbit) continue;
                        fld               = getField();
                        field_items_total = fld.const_dims_total;
                        if(fld.var_dims != null)
                        {
                            if(this.var_dims == null || this.var_dims.Length < fld.var_dims.Length) this.var_dims = new int [fld.var_dims.Length];
                            for(int i = 0, bit = BIT_S; i < fld.var_dims.Length; i++)
                                field_items_total *= this.var_dims[i] = (int) get_bits(bytes, bit -= fld.var_dims[i], fld.var_dims[i]);
                        }
                        break;
                    }
                    BIT          = BIT_S - fld.field_info_bits;
                    BYTE         = BYTE_S;
                    field_item_0 = 0;
                    field_item   = int.MaxValue;
                    field_items  = field_items_total;
                    switch(fld.type)
                    {
                        case 1:
                            item_len = fld.datatype == null ? fld.length : type_len(fld.datatype, BYTE_S);
                            break;
                        case 3:
                            item_len = (int) get_bits(bytes, BIT, fld.length);
                            break;
                        case 5:
                        case 11:
                            item_len = 0;
                            break;
                        case 7:
                            BIT      = BIT_E;
                            item_len = fld.length;
                            break;
                        case 9:
                            item_len = (int) get_bits(bytes, BIT, fld.length);
                            BIT      = BIT_E;
                            break;
                        default:
                            field_item_0 = (int) get_bits(bytes, BIT + fld.sparse_bits, fld.sparse_bits);
                            field_items  = (int) get_bits(bytes, BIT,                   fld.sparse_bits);
                            switch(fld.type)
                            {
                                case 2:
                                    item_len = fld.datatype == null ? fld.length : 0;
                                    break;
                                case 6:
                                case 8:
                                case 12:
                                    item_len = fld.length;
                                    break;
                                case 4:
                                case 10:
                                    item_len = (int) get_bits(bytes, BIT + 2 * fld.sparse_bits, fld.length);
                                    break;
                                default:
                                    Debug.Assert(false);
                                    break;
                            }
                            break;
                    }
                    return true;
                }
                public bool set_item(int item, int length)
                {
                    var fld = getField();
                    if(field_item == item && fld.type != 5) return true;
                    var bit         = BIT;
                    var _field_item = field_item;
                    switch(fld.type)
                    {
                        default:
                            Debug.Assert(false);
                            return false;
                        case 2:
                        case 4:
                            var UDT = fld.datatype != null;
                            var item_bytes = UDT
                                             ? Math.Max(length, fld.datatype.packMinBytes)
                                             : item_len *
                                             fld.size;
                            if(field_items == 0 || item < field_item_0)
                            {
                                if(length < 0) return false;
                                var ins_items = field_items == 0 ? 1 : field_item_0 - item;
                                BIT  = BIT_S - fld.field_info_bits;
                                BYTE = BYTE_S;
                                insert(field_bit, ins_items,
                                       item_bytes);
                                set_bits((ulong)(field_item_0 =  item),      fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits);
                                set_bits((ulong)(field_items  += ins_items), fld.sparse_bits, bytes, BIT_S                       - fld.field_info_bits);
                                BIT = BIT_S - fld.field_info_bits - 1;
                            }
                            else if(item < field_item_0 + field_items)
                            {
                                var item_exists = (bytes[(bit = BIT_S - fld.field_info_bits - 1 - (item - field_item_0)) >> 3] & (1 << (bit & 7))) != 0;
                                if(!item_exists && length < 0) return false;
                                if(item < field_item)
                                {
                                    field_item = field_item_0;
                                    BIT        = BIT_S - fld.field_info_bits - 1;
                                    BYTE       = BYTE_S;
                                }
                                if(UDT)
                                {
                                    for(; field_item < item; field_item++, BIT--)
                                        if((bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
                                            BYTE += type_len(fld.datatype, BYTE);
                                }
                                else
                                    for(; field_item < item; field_item++, BIT--)
                                        if((bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
                                            BYTE += item_bytes;
                                if(item_exists)
                                {
                                    if(UDT && item_len == 0) item_len = type_len(fld.datatype, BYTE);
                                    return true;
                                }
                                Debug.Assert((this.bytes[BIT >> 3] & (1 << (BIT & 7))) == 0);
                                insert(field_bit, 0,
                                       item_bytes);
                            }
                            else
                            {
                                if(length < 0) return false;
                                var ins_items = item + 1 - (field_item_0 + field_items);
                                BIT  = BIT_E;
                                BYTE = BYTE_E;
                                insert(field_bit, ins_items,
                                       item_bytes);
                                set_bits((ulong)(field_items += ins_items), fld.sparse_bits, bytes, BIT_S - fld.field_info_bits);
                                BYTE = BYTE_E - item_bytes;
                                BIT  = BIT_E;
                            }
                            if(UDT) item_len = item_bytes;
                            bytes[BIT >> 3] |= (byte)(1 << (BIT & 7));
                            break;
                        case 5:
                            if(field_items_total - 1 < item) throw new Exception("No room for item=" + item + ". The field_items_total =" + field_items_total);
                            if(length                                                                             < 0 &&
                                    get_bits(bytes, BIT_S - fld.field_info_bits - (item + 1) * fld.length, fld.length) == 0)
                                return false;
                            if(item < field_item)
                            {
                                field_item = 0;
                                BIT        = BIT_S - fld.field_info_bits;
                                BYTE       = BYTE_S;
                                item_len   = (int) get_bits(bytes, BIT -= fld.length, fld.length);
                            }
                            for(;
                                    field_item < item;
                                    BYTE += item_len * fld.size,
                                    item_len = (int) get_bits(bytes, BIT -= fld.length, fld.length),
                                    field_item++)
                                ;
                            if(length < 0) return true;
                            if(item_len == length)
                            {
                                Array.Clear(bytes, BYTE_S, item_len);
                                return true;
                            }
                            var resize = false;
                            if(item_len == 0)
                                insert(field_bit, 0, length * fld.size);
                            else
                            {
                                resize_bytes((length - item_len) * fld.size);
                                resize = true;
                            }
                            set_bits((ulong) length, fld.length, this.bytes, BIT);
                            item_len = length;
                            if(resize) return true;
                            break;
                        case 6:
                            if(field_item_0 <= item && item < field_item_0 + field_items)
                            {
                                var _BYTE = BYTE;
                                if(item < _field_item)
                                {
                                    _field_item = field_item_0;
                                    bit         = BIT_S - fld.field_info_bits - 1;
                                    _BYTE       = BYTE_S;
                                }
                                else
                                {
                                    bit--;
                                    _field_item++;
                                    _BYTE += item_len * fld.size;
                                }
                                for(;; bit--, _field_item++)
                                    if((bytes[bit >> 3] & (1 << (bit & 7))) != 0)
                                    {
                                        bit -= fld.length;
                                        if(_field_item == item)
                                        {
                                            field_item = _field_item;
                                            BYTE       = _BYTE;
                                            BIT        = bit;
                                            item_len   = (int) get_bits(bytes, bit, fld.length);
                                            return true;
                                        }
                                        _BYTE += (int) get_bits(bytes, bit, fld.length) * fld.size;
                                    }
                                    else if(_field_item == item) { break; }
                            }
                            if(length < 0)
                                return false;
                            if(item < field_item_0 || field_items == 0)
                            {
                                BIT  = BIT_S - fld.field_info_bits;
                                BYTE = BYTE_S;
                                var ins_items = field_items == 0 ? 1 : field_item_0 - item;
                                insert(field_bit, ins_items + fld.length, length * fld.size);
                                BIT = BIT_S - fld.field_info_bits - 1;
                                set_bits((ulong)(field_item_0 =  item),      fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits);
                                set_bits((ulong)(field_items  += ins_items), fld.sparse_bits, bytes, BIT_S                       - fld.field_info_bits);
                            }
                            else if(item < field_item_0 + field_items)
                            {
                                if(item < field_item)
                                {
                                    field_item = field_item_0;
                                    BIT        = BIT_S - fld.field_info_bits - 1;
                                    BYTE       = BYTE_S;
                                }
                                for(; field_item < item; BIT--, field_item++)
                                    if((bytes[BIT >> 3] & (1 << (BIT & 7))) != 0)
                                    {
                                        BIT      -= fld.length;
                                        item_len = (int) get_bits(bytes, BIT, fld.length);
                                        BYTE     += item_len * fld.size;
                                    }
                                Debug.Assert((bytes[BIT >> 3] & (1 << (BIT & 7))) == 0);
                                insert(field_bit, fld.length, length * fld.size);
                            }
                            else
                            {
                                var ins_items = item - (field_item_0 + field_items) + 1;
                                BIT  = BIT_E;
                                BYTE = BYTE_E;
                                insert(field_bit, ins_items + fld.length, length * fld.size);
                                BIT  = BIT_E  + fld.length;
                                BYTE = BYTE_E - length * fld.size;
                                set_bits((ulong)(field_items += ins_items), fld.sparse_bits, bytes, BIT_S - fld.field_info_bits);
                            }
                            this.bytes[BIT >> 3] |= (byte)(1 << (BIT & 7));
                            set_bits((ulong)(item_len = length), fld.length, this.bytes, BIT -= fld.length);
                            break;
                        case 8:
                        case 10:
                            var item_bits = item_len * fld.size;
                            if(item < field_item_0 || field_items == 0)
                            {
                                if(length < 0) return false;
                                var ins_items = field_items == 0 ? 1 : field_item_0 - item;
                                BIT  = BIT_S - fld.field_info_bits;
                                BYTE = BYTE_S;
                                insert(field_bit, ins_items +
                                       item_bits,
                                       0);
                                set_bits((ulong)(field_item_0 =  item),      fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits);
                                set_bits((ulong)(field_items  += ins_items), fld.sparse_bits, bytes, BIT_S                       - fld.field_info_bits);
                                BIT = BIT_S - fld.field_info_bits - 1;
                            }
                            else if(item < field_item_0 + field_items)
                            {
                                if(item < _field_item)
                                {
                                    _field_item = field_item_0;
                                    bit         = BIT_S - fld.field_info_bits - 1;
                                }
                                else
                                {
                                    bit--;
                                    _field_item++;
                                }
                                for(;; bit--, _field_item++)
                                    if((bytes[bit >> 3] & (1 << (bit & 7))) != 0)
                                    {
                                        bit -= item_bits;
                                        if(_field_item == item)
                                        {
                                            field_item = item;
                                            BIT        = bit;
                                            return true;
                                        }
                                    }
                                    else if(_field_item == item) break;
                                if(length < 0) return false;
                                Debug.Assert((bytes[BIT >> 3] & (1 << (BIT & 7))) == 0);
                                field_item = item;
                                BIT        = bit;
                                insert(field_bit, item_bits, 0);
                            }
                            else
                            {
                                if(length < 0) return false;
                                var ins_items = item - (field_item_0 + field_items) + 1;
                                BIT = BIT_E;
                                insert(field_bit, ins_items +
                                       item_bits,
                                       0);
                                set_bits((ulong)(field_items += ins_items), fld.sparse_bits, bytes, BIT_S - fld.field_info_bits);
                                BIT = BIT_E + item_bits;
                            }
                            bytes[BIT >> 3] |= (byte)(1 << (BIT & 7));
                            BIT             -= item_bits;
                            break;
                        case 11:
                            if(item < field_items_total)
                            {
                                if(item < _field_item)
                                {
                                    _field_item = -1;
                                    bit         = BIT_S - fld.field_info_bits;
                                }
                                int _items;
                                do
                                {
                                    item_bits =  fld.size * (_items = (int) get_bits(bytes, bit -= fld.length, fld.length));
                                    bit       -= item_bits;
                                    _field_item++;
                                }
                                while(_field_item < item);
                                if(0 < _items)
                                {
                                    BIT        = bit;
                                    item_len   = _items;
                                    field_item = _field_item;
                                    return true;
                                }
                            }
                            if(length < 0)
                                return false;
                            if(field_items_total - 1 < item)
                                throw new Exception("No room for item=" + item + ". The field_items_total =" + field_items_total);
                            if(item < field_item)
                            {
                                field_item = -1;
                                BIT        = BIT_S - fld.field_info_bits;
                            }
                            while(field_item < item)
                            {
                                item_bits = (int) get_bits(bytes, BIT -= fld.length, fld.length) * fld.size;
                                BIT       -= item_bits;
                                field_item++;
                            }
                            if(get_bits(this.bytes, BIT, fld.length) != 0) throw new Exception("Already allocated");
                            insert(field_bit, length * fld.size, 0);
                            set_bits((ulong) length, fld.length, this.bytes, BIT);
                            BIT      -= length * fld.size;
                            item_len =  length;
                            break;
                        case 12:
                            if(item < field_item_0 || field_items == 0)
                            {
                                if(length < 0) return false;
                                var ins_items = field_items == 0 ? 1 : field_item_0 - item;
                                BIT = BIT_S - fld.field_info_bits;
                                insert(field_bit, ins_items  +
                                       fld.length +
                                       length * fld.size,
                                       0);
                                set_bits((ulong)(field_item_0 =  item),      fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits);
                                set_bits((ulong)(field_items  += ins_items), fld.sparse_bits, bytes, BIT_S                       - fld.field_info_bits);
                                BIT = BIT_S - fld.field_info_bits - 1;
                            }
                            else if(item < field_item_0 + field_items)
                            {
                                if(item < _field_item)
                                {
                                    _field_item = field_item_0;
                                    bit         = BIT_S - fld.field_info_bits - 1;
                                }
                                else
                                {
                                    bit--;
                                    _field_item++;
                                }
                                for(int len;; bit--, _field_item++)
                                    if((bytes[bit >> 3] & (1 << (bit & 7))) != 0)
                                    {
                                        bit -= fld.length;
                                        bit -= (len = (int) get_bits(bytes, bit, fld.length)) * fld.size;
                                        if(_field_item == item)
                                        {
                                            field_item = item;
                                            item_len   = len;
                                            BIT        = bit;
                                            return true;
                                        }
                                    }
                                    else if(_field_item == item) break;
                                if(length < 0) return false;
                                Debug.Assert((bytes[BIT >> 3] & (1 << (BIT & 7))) == 0);
                                field_item = item;
                                BIT        = bit;
                                insert(field_bit,                                       fld.length +
                                       length * fld.size, 0);
                            }
                            else
                            {
                                if(length < 0) return false;
                                var ins_items = item - (field_item_0 + field_items) + 1;
                                BIT = BIT_E;
                                insert(field_bit, ins_items  +
                                       fld.length +
                                       length * fld.size,
                                       0);
                                set_bits((ulong)(field_items += ins_items), fld.sparse_bits, bytes, BIT_S - fld.field_info_bits);
                                BIT = BIT_E + length * fld.size + fld.length;
                            }
                            bytes[BIT >> 3] |= (byte)(1 << (BIT & 7));
                            set_bits((ulong)(item_len = length), fld.length, this.bytes, BIT -= fld.length);
                            BIT -= length * fld.size;
                            break;
                    }
                    field_item = item;
                    return false;
                }



                private void resize_bytes(int diff)
                {
                    var cur = this;
                    while(cur.prev != null)
                        cur = (Cursor) cur.prev;
                    if(cur.pack_LAST_BYTE < 0) cur.set_pack_LASTS();
                    var new_bytes = new byte[cur.pack_LAST_BYTE + diff];
                    Array.Copy(bytes, 0, new_bytes, 0, BYTE);
                    if(diff < 0)
                        Array.Copy(bytes, BYTE - diff, new_bytes, BYTE, cur.pack_LAST_BYTE - (BYTE - diff));
                    else
                        Array.Copy(bytes, BYTE, new_bytes, BYTE + diff, cur.pack_LAST_BYTE - BYTE);
                    bytes = new_bytes;
                    for(;;)
                    {
                        if(-1 < cur.pack_LAST_BYTE) cur.pack_LAST_BYTE += diff;
                        cur.BYTE_E += diff;
                        cur.bytes  =  new_bytes;
                        if(cur == this) return;
                        if(-1 < cur.field_bit)
                        {
                            var fld = cur.getField();
                            cur.item_len += diff / fld.size;
                            if(fld.datatype == null)
                                set_bits((ulong) cur.item_len, cur.getField().length, bytes, cur.BIT);
                        }
                        cur = (Cursor) cur.next_;
                    }
                }
                private void insert(int fbit, int bits, int bytes)
                {
                    if(field_bit != fbit)
                    {
                        BIT      = BIT_S  = BIT_E;
                        BYTE     = BYTE_S = BYTE_E;
                        item_len = 0;
                    }
                    if(pack_LAST_BYTE < 0) set_pack_LASTS();
                    int add_to_bits_bytes;
                    var len_bits = meta.BITS_lenINbytes_bits;
                    if(BIT == pack_LAST_BIT && BYTE == pack_LAST_BYTE)
                    {
                        add_to_bits_bytes = len_bits == 0
                                            ? 0
                                            : bits2bytes(fbit + bits - (pack_LAST_BIT - origin * 8) + 1);
                        if(0 < add_to_bits_bytes || 0 < bytes)
                        {
                            resize_bytes(add_to_bits_bytes + bytes);
                            if(0 < add_to_bits_bytes)
                            {
                                copy_bits(this.bytes,     BIT,
                                          BYTE * 8 - BIT, this.bytes, BIT + add_to_bits_bytes * 8);
                                set_0(this.bytes, BIT, add_to_bits_bytes * 8);
                            }
                        }
                    }
                    else
                    {
                        var add_bits_bits = len_bits == 0
                                            ? 0
                                            : bits - (pack_LAST_BIT - (origin * 8 + pack_LAST_field_bit + 1));
                        add_to_bits_bytes =
                            len_bits == 0
                            ? 0
                            : bits2bytes(add_bits_bits);
                        if(0 < add_to_bits_bytes || 0 < bytes)
                        {
                            resize_bytes(add_to_bits_bytes + bytes);
                            if(0 < bits)
                            {
                                copy_bits(this.bytes,     BIT,
                                          BYTE * 8 - BIT, this.bytes, BIT + add_to_bits_bytes * 8);
                                copy_bits(this.bytes,          pack_LAST_BIT,
                                          BIT - pack_LAST_BIT, this.bytes, pack_LAST_BIT + (add_to_bits_bytes << 3) - bits);
                                if(0 < (add_to_bits_bytes << 3) - bits)
                                    set_0(this.bytes, pack_LAST_BIT, (add_to_bits_bytes << 3) - bits);
                                set_0(this.bytes, BIT + (add_to_bits_bytes << 3) - bits, bits);
                            }
                        }
                        else
                        {
                            copy_bits(this.bytes,          pack_LAST_BIT,
                                      BIT - pack_LAST_BIT, this.bytes, pack_LAST_BIT - bits);
                            set_0(this.bytes, BIT - bits, bits);
                        }
                    }
                    if(0 < len_bits && 0 < add_to_bits_bytes)
                    {
                        var old_value = (int) get_bits(this.bytes, origin * 8 + meta.field_0_bit - len_bits, len_bits);
                        set_bits((ulong)(old_value + add_to_bits_bytes), len_bits, this.bytes, origin * 8 + meta.field_0_bit - len_bits);
                    }
                    var zazor_delta = add_to_bits_bytes * 8 - bits;
                    pack_LAST_BIT += zazor_delta;
                    BIT           += add_to_bits_bytes * 8;
                    BYTE          += add_to_bits_bytes;
                    if(fbit == field_bit)
                    {
                        BIT_E  += zazor_delta;
                        BIT_S  += add_to_bits_bytes * 8;
                        BYTE_S += add_to_bits_bytes;
                    }
                    else
                    {
                        BIT_E                            = (BIT_S  = BIT)  - bits;
                        BYTE_E                           = (BYTE_S = BYTE) + bytes;
                        this.bytes[origin + (fbit >> 3)] |= (byte)(1 << (fbit & 7));
                        if(pack_LAST_field_bit < fbit) pack_LAST_field_bit = fbit;
                        field_bit = fbit;
                    }
                }
            }
        }


        public abstract class Channel
        {
            internal const byte BR = 0x55;

            private static readonly ushort[] tab =
            {
                0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548, 53677, 57806,
                61935
            };



            private static ushort crc16(byte data, ushort crc)
            {
                crc = (ushort)(tab[((crc >> 12) ^ (data >> 4)) & 0x0F] ^ (crc << 4));
                return (ushort)(tab[((crc >> 12) ^ (data & 0x0F)) & 0x0F] ^ (crc << 4));
            }

            private struct Flow
            {
                internal readonly Pack.Meta.Field.CursorBase curs;
                internal          Pack.Meta.Field.CursorBase cur;
                internal          ulong                      Uvalue;
                internal          ushort                     crc;

                internal Flow(bool dumb)
                {
                    cur    = curs = new Pack.Meta.Field.CursorBase(null);
                    state  = STATE.STANDBY;
                    mode   = MODE.NONE;
                    Uvalue = 0;
                    crc    = 0;
                }

                internal STATE state;

                internal enum STATE
                {
                    STANDBY,
                    PACK_ID,
                    VARINT,
                    VARINT_BR,
                    BYTES,
                    BYTES_BR
                }

                internal MODE mode;

                internal enum MODE
                {
                    OPTS_INFO,
                    NONE,
                    CRC
                }

                internal Pack.Meta.Field.CursorBase next()
                {
                    start:
                    if(cur.field_bit == -1)
                        switch(cur.item_len)
                        {
                            case 0:
                                if(cur.BIT_E == 0 && cur.meta.fields_reqs != null)
                                {
                                    cur.BIT_E = cur.meta.fields_reqs.Length;
                                    var fld  = cur.meta.fields_reqs[0];
                                    var prev = cur;
                                    cur           = cur.next_ ?? (cur.next_ = new Pack.Meta.Field.CursorBase(cur));
                                    prev.item_len = fld.const_dims_total;
                                    cur.meta      = fld.datatype;
                                    cur.origin    = cur.BYTE_S = cur.BYTE_E = prev.origin;
                                    cur.bytes     = prev.bytes;
                                    cur.field_bit = -1;
                                    cur.item_len  = 0;
                                goto case 0;
                                }
                                cur.BYTE_S = cur.BYTE_E = cur.origin;
                                if(0 < cur.meta._2)
                                {
                                    cur.BYTE_E += cur.meta._2 * (cur.item_len = 2);
                                    state      =  STATE.VARINT;
                                    return cur;
                                }
                            goto case 2;
                            case 2:
                                if(0 < cur.meta._4)
                                {
                                    cur.BYTE_E += cur.meta._4 * (cur.item_len = 4);
                                    state      =  STATE.VARINT;
                                    return cur;
                                }
                            goto case 4;
                            case 4:
                                if(0 < cur.meta._8)
                                {
                                    cur.BYTE_E += cur.meta._8 * (cur.item_len = 8);
                                    state      =  STATE.VARINT;
                                    return cur;
                                }
                            goto case 8;
                            case 8:
                                if(cur.BYTE_S < (cur.BYTE_E = cur.origin + cur.meta.packMinBytes))
                                {
                                    state        = STATE.BYTES;
                                    cur.item_len = 1;
                                    return cur;
                                }
                                goto default;
                            default:
                                if(cur.meta.fields_opts != null)
                                {
                                    if(mode == MODE.OPTS_INFO)
                                    {
                                        mode      = MODE.NONE;
                                        cur.BIT_E = cur.BIT_S = (cur.BYTE_S = cur.BYTE_E) << 3;
                                    }
                                    else
                                    {
                                        var req_last_byte = cur.BYTE_S;
                                        cur.reset();
                                        if(req_last_byte < cur.BYTE_E)
                                        {
                                            cur.BYTE_S   = req_last_byte;
                                            mode         = MODE.OPTS_INFO;
                                            state        = STATE.BYTES;
                                            cur.item_len = 1;
                                            return cur;
                                        }
                                    }
                                    if((cur.field_bit = cur.next_field_bit()) < 0) goto opts_end;
                                    break;
                                }
                                var PREV = cur.prev;
                                if(PREV           == null) return null;
                                if(PREV.field_bit != -1) goto opts_end;
                                if(0                 < --PREV.item_len) cur.origin = cur.BYTE_E;
                                else if(++PREV.BIT_S == PREV.BIT_E) cur            = PREV;
                                else
                                {
                                    var fld = PREV.meta.fields_reqs[PREV.BIT_S];
                                    PREV.item_len = fld.const_dims_total;
                                    cur.meta      = fld.datatype;
                                    cur.origin    = cur.BYTE_S = cur.BYTE_E;
                                }
                                cur.field_bit = -1;
                                cur.item_len  = 0;
                            goto case 0;
                        }
                    else if((cur.field_bit = cur.next_field_bit()) < 0) goto opts_end;
                    next_field:
                    do
                    {
                        var next_fld = cur.getField();
                        if(next_fld.datatype == null)
                        {
                            state = next_fld.varint ? STATE.VARINT : STATE.BYTES;
                            cur.set_E();
                            if(cur.BYTE_S < cur.BYTE_E) return cur;
                        }
                        else
                        {
                            cur.BYTE_S = cur.BYTE_E;
                            cur.BIT_S  = cur.BIT_E;
                            switch(next_fld.type)
                            {
                                case 1:
                                    cur.item_len = next_fld.const_dims_total;
                                    if(next_fld.var_dims != null)
                                        for(int i = 0, bit = cur.BIT_E; i < next_fld.var_dims.Length; i++)
                                            cur.item_len *= (int) get_bits(cur.bytes, bit -= next_fld.var_dims[i], next_fld.var_dims[i]);
                                    cur.BIT_E -= next_fld.field_info_bits;
                                    break;
                                case 2:
                                    cur.BIT_E -= next_fld.field_info_bits;
                                    if((cur.item_len = (int) get_bits(cur.bytes, cur.BIT_E, next_fld.sparse_bits)) == 0) goto next;
                                    cur.BIT_E--;
                                    break;
                            }
                            var prev = cur;
                            cur           = cur.next_ ?? (cur.next_ = new Pack.Meta.Field.CursorBase(cur));
                            cur.origin    = prev.BYTE_E;
                            cur.bytes     = prev.bytes;
                            cur.meta      = next_fld.datatype;
                            cur.field_bit = -1;
                            cur.item_len  = 0;
                            goto start;
                        }
next: ;
                    }
                    while(-1 < (cur.field_bit = cur.next_field_bit()));
                    opts_end:
                    for(var prev = cur.prev; prev != null;)
                    {
                        for(var sparse = prev.getField().type == 4; 0 < --prev.item_len;)
                        {
                            if(sparse && (prev.bytes[--prev.BIT_E >> 3] & (1 << (prev.BIT_E & 7))) == 0) continue;
                            cur.origin    = cur.BYTE_E;
                            cur.field_bit = -1;
                            cur.item_len  = 0;
                            goto start;
                        }
                        prev.BYTE_E = cur.BYTE_E;
                        cur         = prev;
                        if(-1 < (cur.field_bit = cur.next_field_bit())) goto next_field;
                        prev = cur.prev;
                    }
                    return null;
                }
            }

            public abstract class Transmitter
            {
                private readonly int  id_bytes;
                private          Flow flow;

                public Transmitter(int id_bytes)
                {
                    this.id_bytes = id_bytes;
                    flow          = new Flow(true);
                }

                public Action<String> events = null;

                protected internal abstract Pack pullSendingPack();

                public int PacksIntoBytes(byte[] dst, int BYTE, int bytes)
                {
                    var fix = BYTE;
                    for(var cur = flow.cur; 0 < bytes--; BYTE++)
                    {
                        switch(flow.state)
                        {
                            case Flow.STATE.STANDBY:
                                var pack = pullSendingPack();
                                if(pack == null) return BYTE - fix;
                                cur.wrap(pack);
                                cur.field_bit = 8 * (id_bytes - 1);
                                flow.state    = Flow.STATE.PACK_ID;
                                flow.Uvalue   = (ulong) pack.meta.id;
                            goto case Flow.STATE.PACK_ID;
                            case Flow.STATE.PACK_ID:
                                dst[BYTE] = (byte)(0xFF & (flow.Uvalue >> cur.field_bit));
                                if(-1 < (cur.field_bit -= 8)) continue;
                                cur.field_bit = -1;
                                flow.mode     = Flow.MODE.NONE;
                                cur.BIT_E     = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_len = 0;
                                if(cur.meta.fields_opts != null)
                                {
                                    flow.state  = Flow.STATE.VARINT;
                                    flow.Uvalue = (ulong)(cur.type_len(cur.meta, cur.origin) - cur.meta.packMinBytes);
                                    continue;
                                }
                                flow.Uvalue = 0;
                                break;
                            case Flow.STATE.BYTES:
                                dst[BYTE] = cur.bytes[cur.BYTE_S++];
                                if(cur.BYTE_S < cur.BYTE_E) continue;
                                break;
                            case Flow.STATE.VARINT:
                                if((flow.Uvalue & ~0x7FUL) != 0)
                                {
                                    dst[BYTE]   = (byte)((flow.Uvalue & 0x7F) | 0x80);
                                    flow.Uvalue >>= 7;
                                    continue;
                                }
                                dst[BYTE] = (byte) flow.Uvalue;
                                if((cur.BYTE_S += cur.item_len) < cur.BYTE_E)
                                {
                                    flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_len);
                                    continue;
                                }
                                break;
                        }
                        if((cur = flow.next()) != null)
                        {
                            if(flow.state == Flow.STATE.VARINT)
                                flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_len);
                            continue;
                        }
                        flow.state = Flow.STATE.STANDBY;
                        flow.mode  = Flow.MODE.NONE;
                        cur        = flow.curs;
                        for(; cur != null; cur.meta = null, cur.bytes = null, cur = cur.next_) ;
                        cur = flow.curs;
                    }
                    return BYTE - fix;
                }

                public abstract class Advanced : Transmitter
                {
                    protected Advanced(int id_bytes) : base(id_bytes) { }

                    public new int PacksIntoBytes(byte[] dst, int BYTE, int bytes)
                    {
                        var fix = BYTE;
                        var cur = flow.cur;
                        for(int t; 0 < bytes--; t = flow.mode != Flow.MODE.CRC ? flow.crc = crc16(dst[BYTE], flow.crc) : 0, BYTE++)
                        {
                            switch(flow.state)
                            {
                                case Flow.STATE.STANDBY:
                                    var pack = pullSendingPack();
                                    if(pack == null) return BYTE - fix;
                                    cur.wrap(pack);
                                    cur.field_bit = 8 * (id_bytes - 1);
                                    flow.state    = Flow.STATE.PACK_ID;
                                    flow.Uvalue   = (ulong) pack.meta.id;
                                    flow.crc      = 0;
                                    dst[BYTE]     = BR;
                                    continue;
                                case Flow.STATE.PACK_ID:
                                    dst[BYTE] = (byte)(0xFF & (flow.Uvalue >> cur.field_bit));
                                    if(-1 < (cur.field_bit -= 8)) continue;
                                    cur.field_bit = -1;
                                    flow.mode     = Flow.MODE.NONE;
                                    cur.BIT_E     = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_len = 0;
                                    if(cur.meta.fields_opts != null)
                                    {
                                        flow.state  = Flow.STATE.VARINT;
                                        flow.Uvalue = (ulong)(cur.type_len(cur.meta, cur.origin) - cur.meta.packMinBytes);
                                        continue;
                                    }
                                    flow.Uvalue = 0;
                                    break;
                                case Flow.STATE.BYTES:
                                    if(flow.mode == Flow.MODE.CRC)
                                    {
                                        switch(cur.item_len)
                                        {
                                            case 4:
                                                cur.item_len = (dst[BYTE] = (byte)(flow.crc >> 8)) == BR ? 3 : 2;
                                                continue;
                                            case 3:
                                                dst[BYTE]    = BR;
                                                cur.item_len = 2;
                                                continue;
                                            case 2:
                                                if((dst[BYTE] = (byte)(flow.crc & 0xFF)) != BR) break;
                                                cur.item_len = 1;
                                                continue;
                                            case 1:
                                                dst[BYTE] = BR;
                                                break;
                                        }
                                        flow.state = Flow.STATE.STANDBY;
                                        flow.mode  = Flow.MODE.NONE;
                                        cur        = flow.curs;
                                        for(; cur != null; cur.meta = null, cur.bytes = null, cur = cur.next_) ;
                                        cur = flow.curs;
                                        continue;
                                    }
                                    if((flow.Uvalue = dst[BYTE] = cur.bytes[cur.BYTE_S++]) == BR)
                                    {
                                        flow.state = Flow.STATE.BYTES_BR;
                                        continue;
                                    }
                                goto case Flow.STATE.BYTES_BR;
                                case Flow.STATE.BYTES_BR:
                                    flow.state = Flow.STATE.BYTES;
                                    dst[BYTE]  = (byte) flow.Uvalue;
                                    if(cur.BYTE_S < cur.BYTE_E) continue;
                                    break;
                                case Flow.STATE.VARINT:
                                    if((flow.Uvalue & ~0x7FUL) != 0)
                                    {
                                        dst[BYTE]   = (byte)((flow.Uvalue & 0x7F) | 0x80);
                                        flow.Uvalue >>= 7;
                                        continue;
                                    }
                                    if(flow.Uvalue == BR)
                                    {
                                        flow.state = Flow.STATE.VARINT_BR;
                                        dst[BYTE]  = BR;
                                        continue;
                                    }
                                goto case Flow.STATE.VARINT_BR;
                                case Flow.STATE.VARINT_BR:
                                    flow.state = Flow.STATE.VARINT;
                                    dst[BYTE]  = (byte) flow.Uvalue;
                                    if((cur.BYTE_S += cur.item_len) < cur.BYTE_E)
                                    {
                                        flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_len);
                                        continue;
                                    }
                                    break;
                            }
                            if((cur = flow.next()) != null)
                            {
                                if(flow.state == Flow.STATE.VARINT)
                                    flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_len);
                                continue;
                            }
                            cur          = flow.curs;
                            flow.state   = Flow.STATE.BYTES;
                            flow.mode    = Flow.MODE.CRC;
                            cur.item_len = 4;
                        }
                        return BYTE - fix;
                    }
                }
            }

            public abstract class Receiver
            {
                private readonly int id_bytes;

                private          Flow flow;
                private          int  bits;
                private volatile int  time;

                protected Receiver(int id_bytes)
                {
                    this.id_bytes = id_bytes;
                    flow          = new Flow(true);
                }

                protected internal abstract Pack.Meta dispatch(int id, Pack pack);

                public Action<String> events = null;

                public void BytesIntoPacks(byte[] src, int BYTE, int bytes)
                {
                    time = ~0;
                    if(time == 0)
                    {
                        time       = ~0;
                        flow.state = Flow.STATE.STANDBY;
                        events?.Invoke("Receive timeout");
                    }
                    for(var cur = flow.cur; 0 < bytes--; BYTE++)
                    {
                        switch(flow.state)
                        {
                            case Flow.STATE.STANDBY:
                                flow.Uvalue = 0;
                                bits        = 0;
                                flow.state  = Flow.STATE.PACK_ID;
                            goto case Flow.STATE.PACK_ID;
                            case Flow.STATE.PACK_ID:
                                flow.Uvalue = (flow.Uvalue << 8) | src[BYTE];
                                if(++bits < id_bytes) continue;
                                flow.mode = Flow.MODE.NONE;
                                var meta = dispatch((int) flow.Uvalue, null);
                                if(meta == null)
                                {
                                    events?.Invoke("Unrecognized package ID = " + flow.Uvalue);
                                    flow.state = Flow.STATE.STANDBY;
                                    continue;
                                }
                                cur.meta      = meta;
                                flow.Uvalue   = 0;
                                bits          = 0;
                                cur.field_bit = -1;
                                cur.BIT_E     = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_len = 0;
                                if(meta.fields_opts != null)
                                {
                                    flow.state = Flow.STATE.VARINT;
                                    flow.mode  = Flow.MODE.OPTS_INFO;
                                    continue;
                                }
                                else cur.bytes = new byte[meta.packMinBytes];
                                break;
                            case Flow.STATE.BYTES:
                                cur.bytes[cur.BYTE_S++] = src[BYTE];
                                if(cur.BYTE_S < cur.BYTE_E) continue;
                                break;
                            case Flow.STATE.VARINT:
                                flow.Uvalue |= (ulong)((src[BYTE] & 0x7FL) << bits);
                                bits        += 7;
                                if((src[BYTE] & 0x80) != 0) continue;
                                bits = 0;
                                if(flow.mode == Flow.MODE.OPTS_INFO)
                                {
                                    cur.bytes   = new byte[cur.meta.packMinBytes + (int) flow.Uvalue];
                                    flow.Uvalue = 0;
                                    flow.mode   = Flow.MODE.NONE;
                                    break;
                                }
                                set_bytes(flow.Uvalue, cur.item_len, cur.bytes, cur.BYTE_S);
                                flow.Uvalue = 0;
                                if((cur.BYTE_S += cur.item_len) < cur.BYTE_E) continue;
                                break;
                        }
                        if((cur = flow.next()) != null) continue;
                        cur = flow.curs;
                        dispatch(cur.meta.id, cur.unwrap());
                        cur = flow.curs;
                        for(; cur != null; cur.meta = null, cur.bytes = null, cur = cur.next_) ;
                        cur        = flow.curs;
                        flow.state = Flow.STATE.STANDBY;
                        flow.mode  = Flow.MODE.NONE;
                    }
                }

                public abstract class Advanced : Receiver
                {
                    protected Advanced(int id_bytes) : base(id_bytes) { }

                    public new void BytesIntoPacks(byte[] src, int BYTE, int bytes)
                    {
                        time = ~0;
                        if(time == 0)
                        {
                            time       = ~0;
                            flow.state = Flow.STATE.STANDBY;
                            events?.Invoke("Receive timeout");
                        }
                        var cur = flow.cur;
                        for(int t; 0 < bytes--; t = flow.mode != Flow.MODE.CRC ? flow.crc = crc16(src[BYTE], flow.crc) : 0, BYTE++)
                        {
                            switch(flow.state)
                            {
                                case Flow.STATE.STANDBY:
                                    flow.crc    = 0;
                                    flow.Uvalue = 0;
                                    bits        = 0;
                                    if(src[BYTE] == BR) flow.state = Flow.STATE.PACK_ID;
                                    continue;
                                case Flow.STATE.PACK_ID:
                                    if(src[BYTE] == BR)
                                    {
                                        events?.Invoke(" After BR expect helper ID but got +BR");
                                        flow.state = Flow.STATE.STANDBY;
                                        continue;
                                    }
                                    flow.Uvalue = (flow.Uvalue << 8) | src[BYTE];
                                    if(++bits < id_bytes) continue;
                                    flow.mode = Flow.MODE.NONE;
                                    var meta = dispatch((int) flow.Uvalue, null);
                                    if(meta == null)
                                    {
                                        events?.Invoke("Unrecognized package ID = " + flow.Uvalue);
                                        flow.state = Flow.STATE.STANDBY;
                                        continue;
                                    }
                                    cur.meta      = meta;
                                    flow.Uvalue   = 0;
                                    bits          = 0;
                                    cur.field_bit = -1;
                                    cur.BIT_E     = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_len = 0;
                                    if(meta.fields_opts != null)
                                    {
                                        flow.state = Flow.STATE.VARINT;
                                        flow.mode  = Flow.MODE.OPTS_INFO;
                                        continue;
                                    }
                                    else cur.bytes = new byte[meta.packMinBytes];
                                    break;
                                case Flow.STATE.BYTES:
                                    if(src[BYTE] == BR)
                                    {
                                        flow.state = Flow.STATE.BYTES_BR;
                                        continue;
                                    }
                                goto case Flow.STATE.BYTES_BR;
                                case Flow.STATE.BYTES_BR:
                                    if(flow.state == Flow.STATE.BYTES_BR)
                                    {
                                        if(src[BYTE] != BR)
                                        {
                                            events?.Invoke("waiting for second BR but got " + src[BYTE]);
                                            flow.state = Flow.STATE.STANDBY;
                                            continue;
                                        }
                                        flow.state = Flow.STATE.BYTES;
                                    }
                                    if(flow.mode == Flow.MODE.CRC)
                                        switch(cur.item_len)
                                        {
                                            case 2:
                                                flow.Uvalue  = (ulong)(src[BYTE] << 8);
                                                cur.item_len = 1;
                                                continue;
                                            case 1:
                                                if((flow.Uvalue | src[BYTE]) == flow.crc)
                                                    dispatch(cur.meta.id, cur.unwrap());
                                                else events?.Invoke("CRC error");
                                                cur = flow.curs;
                                                for(; cur != null; cur.meta = null, cur.bytes = null, cur = cur.next_) ;
                                                flow.state = Flow.STATE.STANDBY;
                                                flow.mode  = Flow.MODE.NONE;
                                                continue;
                                        }
                                    cur.bytes[cur.BYTE_S++] = src[BYTE];
                                    if(cur.BYTE_S < cur.BYTE_E) continue;
                                    break;
                                case Flow.STATE.VARINT:
                                    if(src[BYTE] == BR)
                                    {
                                        flow.state = Flow.STATE.VARINT_BR;
                                        continue;
                                    }
                                goto case Flow.STATE.VARINT_BR;
                                case Flow.STATE.VARINT_BR:
                                    if(flow.state == Flow.STATE.VARINT_BR)
                                    {
                                        if(src[BYTE] != BR)
                                        {
                                            events?.Invoke("waiting for second BR but got " + src[BYTE]);
                                            flow.state = Flow.STATE.STANDBY;
                                            continue;
                                        }
                                        flow.state = Flow.STATE.VARINT;
                                    }
                                    flow.Uvalue |= (ulong)((src[BYTE] & 0x7FL) << bits);
                                    bits        += 7;
                                    if((src[BYTE] & 0x80) != 0) continue;
                                    bits = 0;
                                    if(flow.mode == Flow.MODE.OPTS_INFO)
                                    {
                                        cur.bytes   = new byte[cur.meta.packMinBytes + (int) flow.Uvalue];
                                        flow.Uvalue = 0;
                                        flow.mode   = Flow.MODE.NONE;
                                        break;
                                    }
                                    set_bytes(flow.Uvalue, cur.item_len, cur.bytes, cur.BYTE_S);
                                    flow.Uvalue = 0;
                                    if((cur.BYTE_S += cur.item_len) < cur.BYTE_E) continue;
                                    break;
                            }
                            if((cur = flow.next()) != null) continue;
                            cur          = flow.curs;
                            flow.state   = Flow.STATE.BYTES;
                            flow.mode    = Flow.MODE.CRC;
                            cur.item_len = 2;
                        }
                    }
                }
            }
        }
    }
}