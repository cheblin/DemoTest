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
package org.unirail;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import static java.lang.Character.MAX_SURROGATE;
import static java.lang.Character.MIN_SURROGATE;

public abstract class AdHoc {
	
	final static         char[] Ol = {0, 1, 3, 7, 15, 31, 63, 127, 255};
	private final static char[] lO = {0, 128, 192, 224, 240, 248, 252, 254, 255};
	
	public static long get_bytes( byte[] src, int BYTE, int bytes ) {
		int hi = 0, lo = 0;
		switch (bytes)
		{
			case 8:
				hi |= (src[BYTE + 7] & 0xFF) << 24;
			case 7:
				hi |= (src[BYTE + 6] & 0xFF) << 16;
			case 6:
				hi |= (src[BYTE + 5] & 0xFF) << 8;
			case 5:
				hi |= src[BYTE + 4] & 0xFF;
			case 4:
				lo |= (src[BYTE + 3] & 0xFF) << 24;
			case 3:
				lo |= (src[BYTE + 2] & 0xFF) << 16;
			case 2:
				lo |= (src[BYTE + 1] & 0xFF) << 8;
			case 1:
				lo |= src[BYTE] & 0xFF;
		}
		return (hi & 0xFFFFFFFFL) << 32 | lo & 0xFFFFFFFFL;
	}
	
	public static int set_bytes( long src, int bytes, byte[] dst, int BYTE ) {
		int hi = (int) (src >>> 32), lo = (int) (src & 0xFFFFFFFFL);
		switch (bytes)
		{
			case 8:
				dst[BYTE + 7] = (byte) (hi >>> 24);
			case 7:
				dst[BYTE + 6] = (byte) (hi >>> 16);
			case 6:
				dst[BYTE + 5] = (byte) (hi >>> 8);
			case 5:
				dst[BYTE + 4] = (byte) (hi & 0xFF);
			case 4:
				dst[BYTE + 3] = (byte) (lo >>> 24);
			case 3:
				dst[BYTE + 2] = (byte) (lo >>> 16);
			case 2:
				dst[BYTE + 1] = (byte) (lo >>> 8);
			case 1:
				dst[BYTE] = (byte) (lo & 0xFF);
		}
		return BYTE + bytes;
	}
	
	public static void set_0( byte[] dst, int bit, int bits ) {
		int dst_byte = bit >> 3;
		bit &= 7;
		if (8 < bit + bits)
		{
			if (0 < bit)
			{
				dst[dst_byte] &= Ol[bit];
				if ((bits -= 8 - bit) == 0) return;
				dst_byte++;
			}
			if (0 < (bits & 7)) dst[dst_byte + (bits >> 3)] &= lO[8 - (bits & 7)];
			if ((bits >>= 3) == 0) return;
			for (int i = dst_byte + bits; dst_byte <= --i; )
			     dst[i] = 0;
		}
		else dst[dst_byte] &= Ol[bit] | lO[8 - (bit + bits)];
	}
	
	public static void set_bits( long src, int bits, byte[] dst, int bit ) {
		int dst_byte = bit >> 3;
		bit &= 7;
		if (8 < bit + bits)
		{
			if (0 < bit)
			{
				dst[dst_byte] = (byte) (dst[dst_byte] & Ol[bit] |
				                        (src & Ol[
						                               8 - bit
						                               ]) << bit);
				dst_byte++;
				src >>>= 8 - bit;
				bits -= 8 - bit;
			}
			for (int BYTE = 0, bytes = bits >> 3; BYTE < bytes; BYTE++, src >>>= 8)
			     dst[dst_byte++] = (byte) (src & 0xFF);
			if (0 < (bits &= 7))
				dst[dst_byte] = (byte) (dst[dst_byte] & lO[8 - bits]
				                        |
				                        src & Ol[bits]);
		}
		else dst[dst_byte] = (byte) (dst[dst_byte] & (Ol[bit] | lO[8 - bit - bits]) | (src & Ol[bits]) << bit);
	}
	
	
	public static void copy_bits( byte[] src, int src_bit, int bits, byte[] dst, int dst_bit ) {
		if (bits < 1 || src == dst && src_bit == dst_bit) return;
		int count = bits >> 6;
		bits &= 0x3F;
		if (src == dst && src_bit < dst_bit)
		{
			
			src_bit += count * 64;
			dst_bit += count * 64;
			if (0 < bits) set_bits( get_bits( src, src_bit, bits ), bits, dst, dst_bit );
			for (; 0 < count--; src_bit -= 64, dst_bit -= 64, set_bits( get_bits( src, src_bit, 64 ), 64, dst, dst_bit )) ;
		}
		else
		{
			for (; 0 < count; set_bits( get_bits( src, src_bit, 64 ), 64, dst, dst_bit ), src_bit += 64, dst_bit += 64, count--) ;
			if (0 < bits) set_bits( get_bits( src, src_bit, bits ), bits, dst, dst_bit );
		}
	}
	
	public static int first_1( byte[] bytes, int bit, int bits, boolean existence ) {
		if (bits < 1) return -1;
		int _1BYTE = bit >> 3;
		int v      = bytes[_1BYTE] & 0xFF;
		bit &= 7;
		if (bits == 1) return (v & 1 << bit) == 0 ? -1 : 0;
		int add = 0;
s:
		{
			if (0 < bit)
			{
				if (0 < (v >>= bit))
				{
					if (bit + bits < 8 && (v & Ol[bits]) == 0) return -1;
					break s;
				}
				if (bit + bits < 8) return -1;
				bits -= add = 8 - bit;
				_1BYTE++;
			}
			else if (bits < 9)
				if (v == 0 || (v & Ol[bits]) == 0) return -1;
				else break s;
			final int last = _1BYTE + (bits >> 3);
			for (int BYTE = _1BYTE; BYTE < last; BYTE++)
				if (0 < (v = bytes[BYTE] & 0xFF))
				{
					add += BYTE - _1BYTE << 3;
					break s;
				}
			if ((bits &= 7) == 0 || (v = bytes[last] & 0xFF & Ol[bits]) == 0) return -1;
			add += last - _1BYTE << 3;
		}
		if (existence) return Integer.MAX_VALUE;
		for (int i = 0; ; i++)
			if ((v >> i & 1) == 1) return add + i;
	}
	
	
	public static int bits2bytes( int bits ) { return bits < 1 ? 0 : 1 + (bits - 1 >> 3); }
	
	public static long get_bits( byte[] src, int bit, int bits ) {
		int src_byte = bit >> 3;
		bit &= 7;
		if (64 < bits) bits = 64;
		else if (bit + bits < 9) return (src[src_byte] & 0xFF) >> bit & Ol[bits];
		long dst = 0;
		for (int i = 0, last = bit + bits >> 3 << 3; i < last; i += 8)
		     dst |= (src[src_byte++] & 0xFFL) << i;
		dst >>>= bit;
		bit = bit + bits & 7;
		if (0 < bit) dst |= (long) (src[src_byte] & Ol[bit]) << bits - bit;
		return dst;
	}
	
	
	public static class Pack {
		public final Meta   meta;
		public       byte[] bytes;
		
		public Pack( Meta meta ) { this.meta = meta; }
		
		public static class Meta {
			public final int id;
			
			private final int _2;
			private final int _4;
			private final int _8;
			
			public final  int  packMinBytes;
			private final char BITS_lenINbytes_bits;
			public final  char nesting_max;
			
			private final int     field_0_bit;
			public final  Field[] fields;
			
			
			public Meta( int id )                                                                             { this( id, 0, 0, 0, 0, 1, 0, 0, 0 ); }
			
			public Meta( int id, int _2, int _4, int _8, int packMinBytes, int nesting_max, int field_0_bit ) { this( id, _2, _4, _8, packMinBytes, nesting_max, field_0_bit, 0, 0 ); }
			
			public Meta( int id, int _2, int _4, int _8, int packMinBytes, int nesting_max, int field_0_bit, int BITS_lenINbytes_bits, int fields ) {
				this.id                   = id;
				this._2                   = _2;
				this._4                   = _4;
				this._8                   = _8;
				this.BITS_lenINbytes_bits = (char) BITS_lenINbytes_bits;
				this.nesting_max          = (char) nesting_max;
				this.field_0_bit          = field_0_bit;
				this.fields               = 0 < fields ? new Field[fields] : null;
				this.packMinBytes         = packMinBytes;
			}
			
			
			public static class Field {
				private final byte    type;
				private final boolean varint;
				private final int     length;
				private final byte    size;
				public final  Meta[]  datatypes;
				
				public final int[] var_dims;
				
				
				private final int const_dims_total;
				
				
				private final char field_info_bits;
				private final char sparse_bits;
				
				public Field( int type, boolean varint, int length, int size, int const_dims_total, int field_info_bits, int sparse_bits, Meta[] datatypes, int... var_dims ) {
					this.type             = (byte) type;
					this.varint           = varint;
					this.length           = length;
					this.size             = (byte) size;
					this.const_dims_total = const_dims_total;
					this.field_info_bits  = (char) field_info_bits;
					this.sparse_bits      = (char) sparse_bits;
					this.datatypes        = datatypes;
					this.var_dims         = var_dims != null && 0 < var_dims.length ? var_dims : null;
				}
				
				
				static class CursorBase {
					public byte[] bytes;
					Meta meta;
					
					public boolean last() {return next_ == null;}
					
					CursorBase next_;
					
					CursorBase prev;
					
					public CursorBase() {
					}
					
					public CursorBase( CursorBase prev, int nested_max ) {
						
						this.prev = prev;
						next_     = 1 < nested_max ? new CursorBase( this, nested_max - 1 ) : null;
					}
					
					public void init( Meta meta ) {
						this.meta = meta;
						bytes     = new byte[meta.packMinBytes];
						reset();
					}
					
					public int origin;
					
					public int item_type = 0;
					
					
					public int BIT_S = -1, BIT_E;
					
					int BYTE_S = -1, BYTE_E = -1;
					
					public int field_bit;
					
					public void wrap( Pack src ) {
						origin    = 0;
						bytes     = src.bytes;
						meta      = src.meta;
						src.bytes = null;
						reset();
					}
					
					int next_field_bit() {
						if (field_bit - meta.field_0_bit < meta.fields.length - 1)
						{
							int bit = origin * 8 + (field_bit < 0 ? meta.field_0_bit : field_bit + 1);
							int i   = first_1( bytes, bit, Math.min( BIT_E - bit, meta.fields.length ), false );
							return i < 0 ? -1 : i + bit - origin * 8;
						}
						
						return -1;
					}
					
					public Field getField() { return meta.fields[field_bit - meta.field_0_bit]; }
					
					boolean reset() {
						field_bit = -1;
						final char len_bits = meta.BITS_lenINbytes_bits;
						BYTE_E    = BYTE_S = origin + meta.packMinBytes + (len_bits == 0 ? 0 : (int) get_bits( bytes, origin * 8 + meta.field_0_bit - len_bits, len_bits ));
						BIT_E     = BIT_S = BYTE_E << 3;
						item_type = 0;
						return true;
					}
					
					
					int type_len( Meta TYPE, int BYTE ) {
						if (TYPE.fields == null) return TYPE.packMinBytes;
						int  bit_0    = BYTE * 8 + TYPE.field_0_bit;
						char len_bits = TYPE.BITS_lenINbytes_bits;
						
						int LAST_BYTE = BYTE + TYPE.packMinBytes + (len_bits == 0 ? 0 : (int) get_bits( bytes, bit_0 - len_bits, len_bits ));
						
						int fb = first_1( bytes, bit_0, Math.min( LAST_BYTE * 8 - bit_0, meta.fields.length ), false );
						if (fb == -1) return LAST_BYTE - BYTE;
						fb += TYPE.field_0_bit;
						
						int
								_BIT_E = BIT_E,
								_BIT_S = BIT_S,
								_BYTE_S = BYTE_S,
								_BYTE_E = BYTE_E,
								_item_type = item_type,
								_origin = origin,
								_field_bit = field_bit;
						Meta _meta = meta;
						meta   = TYPE;
						origin = BYTE;
						BYTE_E = BYTE_S = LAST_BYTE;
						BIT_E  = BIT_S = LAST_BYTE << 3;
						do
						{
							field_bit = fb;
							BIT_S     = BIT_E;
							BYTE_S    = BYTE_E;
							Field fld = getField();
							if (0 < fld.length)
								if (0 < fld.size)
									BYTE_E += fld.const_dims_total * fld.length * fld.size;
								else
									BIT_E += fld.const_dims_total * fld.length * fld.size;
							else
								set_E( fld );
						} while (-1 < (fb = next_field_bit()));
						
						int ret = BYTE_E - BYTE;
						BIT_E     = _BIT_E;
						BIT_S     = _BIT_S;
						BYTE_S    = _BYTE_S;
						BYTE_E    = _BYTE_E;
						item_type = _item_type;
						origin    = _origin;
						field_bit = _field_bit;
						meta      = _meta;
						return ret;
					}
					
					
					void set_E( Field fld ) {
						int bit = BIT_S;
						
						int count = fld.const_dims_total;
						if (fld.var_dims != null)
							for (int i = 0; i < fld.var_dims.length; i++)
							     count *= (int) get_bits( bytes, bit -= fld.var_dims[i], fld.var_dims[i] );
						
						switch (fld.type)
						{
							case 1:
								BIT_E -= fld.field_info_bits;
								BYTE_E += count * (fld.datatypes == null ? -fld.length * fld.size : type_len( fld.datatypes[item_type = 0], BYTE_S ));
								break;
							case 3:
								BIT_E -= fld.field_info_bits;
								count *= (int) get_bits( bytes, BIT_E, -fld.length );
								
								BYTE_E += fld.datatypes == null ? count * fld.size : type_len( fld.datatypes[item_type = count], BYTE_E );
								break;
							case 5:
								
								BIT_E -= fld.field_info_bits;
								if (fld.datatypes == null)
								{
									int all_arrays_sum = 0;
									while (0 < count--)
										all_arrays_sum += (int) get_bits( bytes, BIT_E -= -fld.length, -fld.length );
									BYTE_E += all_arrays_sum * fld.size;
								}
								else
									while (0 < count--)
										if (0 < (item_type = (int) get_bits( bytes, BIT_E -= -fld.length, -fld.length )))
											BYTE_E += type_len( fld.datatypes[--item_type], BYTE_E );
								
								break;
							case 7:
								
								BIT_E -= fld.field_info_bits + count * -fld.length * fld.size;
								break;
							case 9:
								BIT_E -= fld.field_info_bits;
								count *= (int) get_bits( bytes, BIT_E, -fld.length );
								
								BIT_E -= count * fld.size;
								break;
							case 11:
								
								BIT_E -= fld.field_info_bits;
								while (0 < count--)
								{
									int bits = (int) get_bits( bytes, BIT_E -= -fld.length, -fld.length ) * fld.size;
									BIT_E -= bits;
								}
								
								break;
							default:
								if ((count = (int) get_bits( bytes, BIT_E -= fld.field_info_bits, fld.sparse_bits )) == 0) return;
								bit = BIT_E;
								switch (fld.type)
								{
									case 2:
										BIT_E -= count;
										while (BIT_E < bit--)
											if ((bytes[bit >> 3] & 1 << (bit & 7)) == 0)
												count--;
										BYTE_E += -fld.length * count * fld.size;
										break;
									case 4:
										BIT_E -= count;
										count *= (int) get_bits( bytes, bit + 2 * fld.sparse_bits, -fld.length );
										while (BIT_E < bit--)
											if ((bytes[bit >> 3] & 1 << (bit & 7)) == 0)
												count--;
										BYTE_E += count * fld.size;
										break;
									case 6:
										int all_arrays_sum = 0;
										while (0 < count--)
											if ((bytes[--bit >> 3] & 1 << (bit & 7)) != 0)
												all_arrays_sum += (int) get_bits( bytes, bit -= -fld.length, -fld.length );
										BIT_E = bit;
										BYTE_E += all_arrays_sum * fld.size;
										break;
									case 8:
										
										for (int bits = -fld.length * fld.size; 0 < count--; )
											if ((bytes[--bit >> 3] & 1 << (bit & 7)) != 0)
												bit -= bits;
										BIT_E = bit;
										break;
									case 10:
										
										for (int bits = (int) get_bits( bytes, bit + 2 * fld.sparse_bits, -fld.length ) * fld.size; 0 < count--; )
											if ((bytes[--bit >> 3] & 1 << (bit & 7)) != 0)
												bit -= bits;
										BIT_E = bit;
										break;
									case 12:
										while (0 < count--)
											if ((bytes[--bit >> 3] & 1 << (bit & 7)) != 0)
											{
												int len = (int) get_bits( bytes, bit -= -fld.length, -fld.length );
												bit -= len * fld.size;
											}
										
										BIT_E = bit;
										break;
									default:
										assert false;
										return;
								}
						}
					}
					
					public Pack unwrap() {
						if (meta == null) return null;
						
						Pack ret = new Pack( meta );
						ret.bytes = bytes;
						
						meta  = null;
						bytes = null;
						
						return ret;
					}
					
					public final boolean equal( CursorBase other, int len ) {
						if (len == 0) return false;
						for (; 0 < --len; ) if (this.bytes[origin + len] != other.bytes[other.origin + len]) return false;
						return true;
					}
				}
			}
		}
		
		public static class Cursor extends Meta.Field.CursorBase {
			
			public final int[] var_dims;
			
			public int BIT = -1;
			public int BYTE;
			public int item_len;
			int pack_LAST_BIT;
			int pack_LAST_BYTE;
			int pack_LAST_field_bit;
			int field_item_0;
			int field_item;
			int field_items;
			int field_items_total;
			
			
			public Cursor( Cursor prev, int nested_max, int var_dim_len ) {
				super();
				var_dims  = 0 < var_dim_len ? new int[var_dim_len] : null;
				this.prev = prev;
				next_     = 1 < nested_max ? new Cursor( this, nested_max - 1, var_dim_len ) : null;
			}
			
			protected int set_pack_LASTS() {
				int fb = next_field_bit();
				if (fb < 0)
				{
					pack_LAST_BYTE      = BYTE_E;
					pack_LAST_BIT       = BIT_E;
					pack_LAST_field_bit = field_bit;
					return pack_LAST_BYTE;
				}
				
				int
						BIT_E_FX = BIT_E,
						BIT_S_FX = BIT_S,
						BYTE_S_FX = BYTE_S,
						BYTE_E_FX = BYTE_E,
						item_type_FX = item_type,
						field_bit_FX = field_bit;
				do
				{
					BIT_S     = BIT_E;
					BYTE_S    = BYTE_E;
					field_bit = fb;
					Meta.Field fld = getField();
					if (0 < fld.length)
						if (0 < fld.size)
							BYTE_E += fld.const_dims_total * fld.length * fld.size;
						else
							BIT_E += fld.const_dims_total * fld.length * fld.size;
					else
						set_E( fld );
				} while (-1 < (fb = next_field_bit()));
				
				
				pack_LAST_BYTE      = BYTE_E;
				pack_LAST_BIT       = BIT_E;
				pack_LAST_field_bit = field_bit;
				BIT_E               = BIT_E_FX;
				BIT_S               = BIT_S_FX;
				BYTE_S              = BYTE_S_FX;
				BYTE_E              = BYTE_E_FX;
				item_type           = item_type_FX;
				field_bit           = field_bit_FX;
				return pack_LAST_BYTE;
			}
			
			public int length() { return (-1 < pack_LAST_BYTE ? pack_LAST_BYTE : set_pack_LASTS()) - origin; }
			
			
			public Cursor next( int origin ) {
				next_.origin = origin;
				next_.bytes  = bytes;
				next_.meta   = getField().datatypes[item_type];
				next_.reset();
				return (Cursor) next_;
			}
			
			
			@Override boolean reset() {
				super.reset();
				if (meta.fields == null ||
				    next_field_bit() < 0)
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
				field_item   = Integer.MAX_VALUE;
				field_items  = 0;
				return true;
			}
			
			@Override public Pack unwrap() {
				Pack ret = super.unwrap();
				for (Meta.Field.CursorBase cur = this; (cur = cur.next_) != null; cur.bytes = null, cur.meta = null) ;
				return ret;
			}
			
			@Override public void wrap( Pack src ) {
				origin = 0;
				bytes  = src.bytes;
				meta   = src.meta;
				reset();
			}
			
			
			public boolean set_field( int fbit, int each_item_size, int... var_dims ) {
				if (field_bit == fbit) return true;
				if (each_item_size < 0 &&
				    (BIT_E <= origin * 8 + fbit ||
				     meta.fields.length <= fbit - meta.field_0_bit ||
				     (bytes[origin + (fbit >> 3)] & 1 << (fbit & 7)) == 0)) return false;
				
				Meta.Field fld;
				if (fbit < field_bit) reset();

insert_field:
				{
					for (; ; )
					{
						int next = next_field_bit();
						if (fbit < next ||
						    next == -1)
							break;
						BIT_S     = BIT_E;
						BYTE_S    = BYTE_E;
						field_bit = next;
						fld       = getField();
						if (0 < fld.length)
							if (0 < fld.size)
								BYTE_E += fld.const_dims_total * fld.length * fld.size;
							else
								BIT_E += fld.const_dims_total * fld.length * fld.size;
						else set_E( fld );
						if (field_bit < fbit) continue;
						
						
						field_items_total = fld.const_dims_total;
						if (fld.var_dims != null)
							for (int i = 0, bit = BIT_S; i < fld.var_dims.length; i++)
							     field_items_total *= this.var_dims[i] = (int) get_bits( bytes, bit -= fld.var_dims[i], fld.var_dims[i] );
						
						
						break insert_field;
					}
					
					if (each_item_size < 0) return false;
					
					
					fld = meta.fields[fbit - meta.field_0_bit];
					
					if (fld.datatypes != null && next_ == null) return false;
					
					if (fld.type == 0)
					{
						if (0 < fld.size)
						{
							
							insert( fbit, 0, fld.const_dims_total * fld.length * fld.size );
							BYTE_E = (BYTE = BYTE_S) + fld.const_dims_total * (item_len = fld.length) * fld.size;
						}
						else
						{
							insert( fbit, fld.const_dims_total * fld.length * -fld.size, 0 );
							BIT = BIT_E = BIT_S + fld.const_dims_total * (item_len = fld.length) * fld.size;
						}
						
						return true;
					}
					
					int total = fld.const_dims_total;
					if (fld.var_dims != null)
						for (int i = 0; i < fld.var_dims.length; i++)
						     total *= this.var_dims[i] = var_dims[i];
					
					switch (fld.type)
					{
						case 1:
							insert( fbit, fld.field_info_bits, total * (fld.datatypes == null ? -fld.length * fld.size : Math.max( each_item_size, fld.datatypes[0].packMinBytes )) );
							item_type = 0;
							break;
						case 2:
						case 6:
						case 8:
						case 12:
							insert( fbit, fld.field_info_bits, 0 );
							break;
						case 3:
							int fix = item_type;
							insert( fbit, fld.field_info_bits, total * (fld.datatypes == null ? each_item_size * fld.size : (each_item_size = Math.max( each_item_size, fld.datatypes[fix].packMinBytes ))) );
							
							set_bits( fld.datatypes == null ? each_item_size : fix, -fld.length, bytes, BIT_S - fld.field_info_bits );
							item_type = fix;
							break;
						case 4:
							insert( fbit, fld.field_info_bits, 0 );
							set_bits( each_item_size, -fld.length, bytes, BIT_S - fld.field_info_bits + 2 * fld.sparse_bits );
							break;
						case 5:
						case 11:
							insert( fbit, fld.field_info_bits + total * -fld.length, 0 );
							break;
						case 7:
							
							insert( fbit, fld.field_info_bits + total * -fld.length * fld.size, 0 );
							break;
						case 9:
							
							insert( fbit, fld.field_info_bits + total * each_item_size * fld.size, 0 );
							set_bits( each_item_size, -fld.length, bytes, BIT_S - fld.field_info_bits );
							break;
						case 10:
							insert( fbit, fld.field_info_bits, 0 );
							set_bits( each_item_size, -fld.length, bytes, BIT_S - fld.field_info_bits + 2 * fld.sparse_bits );
							break;
						default:
							assert false;
							break;
					}
					
					field_items_total = total;
					if (fld.var_dims != null)
						for (int i = 0, bit = BIT_S; i < var_dims.length; i++)
						     set_bits( var_dims[i], fld.var_dims[i], bytes, bit -= fld.var_dims[i] );
					
					
					if (0 < fld.length)
						if (0 < fld.size)
							BYTE_E += fld.const_dims_total * fld.length * fld.size;
						else
							BIT_E += fld.const_dims_total * fld.length * fld.size;
					else set_E( fld );
				}
				
				
				switch (fld.type)
				{
					case 0:
						BIT = BIT_E;
						break;
					case 1:
						item_len = fld.datatypes == null ? -fld.length : type_len( fld.datatypes[item_type = 0], BYTE_S );
						BIT = BIT_E;
						break;
					case 3:
						item_len = (int) get_bits( bytes, BIT = BIT_S - fld.field_info_bits, -fld.length );
						if (fld.datatypes != null)
							item_len = type_len( fld.datatypes[item_type = item_len], BYTE_S );
						break;
					case 5:
					case 11:
						item_len = 0;
						BIT = BIT_S - fld.field_info_bits;
						field_item_0 = 0;
						field_item = Integer.MAX_VALUE;
						field_items = field_items_total;
						break;
					case 7:
						item_len = -fld.length;
						field_item_0 = 0;
						BIT = BIT_E;
						field_item = (field_items = field_items_total) == 0 ? Integer.MAX_VALUE : 0;
						break;
					case 9:
						item_len = (int) get_bits( bytes, BIT = BIT_S - fld.field_info_bits, -fld.length );
						field_item_0 = 0;
						BIT = BIT_E;
						field_item = (field_items = field_items_total) == 0 ? Integer.MAX_VALUE : 0;
						break;
					default:
						BIT = BIT_S - fld.field_info_bits;
						switch (fld.type)
						{
							case 2:
							case 6:
							case 8:
							case 12:
								item_len = -fld.length;
								BIT = BIT_S - fld.field_info_bits;
								break;
							case 4:
							case 10:
								item_len = (int) get_bits( bytes, (BIT = BIT_S - fld.field_info_bits) + 2 * fld.sparse_bits, -fld.length );
								break;
							default:
								assert false;
								break;
						}
						
						field_item = Integer.MAX_VALUE;
						field_items = (int) get_bits( bytes, BIT, fld.sparse_bits );
						field_item_0 = (int) get_bits( bytes, BIT + fld.sparse_bits, fld.sparse_bits );
						break;
				}
				
				BYTE = BYTE_S;
				return true;
			}
			
			
			public boolean set_item( int item, int length ) {
				final Meta.Field fld = getField();
				if (field_item == item && fld.type != 5) return true;
				int bit         = BIT;
				int _field_item = field_item;
				int len_bits    = -fld.length;
				switch (fld.type)
				{
					default:
						
						
						assert false;
						return false;
					case 2:
					case 4:
						if (field_items == 0 || item < field_item_0 || field_item_0 + field_items <= item || (bytes[(bit = BIT_S - fld.field_info_bits - 1 - (item - field_item_0)) >> 3] & 1 << (bit & 7)) == 0)
						{
							if (length < 0)
								return false;
							if (item < field_item_0 || field_items == 0)
							{
								BIT  = BIT_S - fld.field_info_bits;
								BYTE = BYTE_S;
								int ins_items = field_items == 0 ? 1 : field_item_0 - item;
								insert( field_bit, ins_items, item_len * fld.size );
								BIT = BIT_S - fld.field_info_bits - 1;
								set_bits( field_item_0 = item, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits );
								set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
							}
							else if (item < field_item_0 + field_items)
							{
								
								
								if (item < field_item)
								{
									field_item = field_item_0;
									BIT        = BIT_S - fld.field_info_bits - 1;
									BYTE       = BYTE_S;
								}
								else
								{
									BIT--;
									field_item++;
									BYTE += item_len * fld.size;
								}
								
								for (int bytes = item_len * fld.size; field_item < item; field_item++, BIT--)
									if ((this.bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
										BYTE += bytes;
								assert( this.bytes[BIT >> 3] & 1 << (BIT & 7) ) == 0;
								insert( field_bit, 0, item_len * fld.size );
							}
							else
							{
								int ins_items = item + 1 - (field_item_0 + field_items);
								BIT  = BIT_E;
								BYTE = BYTE_E;
								insert( field_bit, ins_items, item_len * fld.size );
								
								BIT  = BIT_E;
								BYTE = BYTE_E - item_len * fld.size;
								set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
							}
							
							this.bytes[BIT >> 3] |= (byte) (1 << (BIT & 7));
							break;
						}
						
						
						if (item < field_item)
						{
							field_item = field_item_0;
							BIT        = BIT_S - fld.field_info_bits - 1;
							BYTE       = BYTE_S;
						}
						else
						{
							BIT--;
							field_item++;
							BYTE += item_len * fld.size;
						}
						
						for (int bytes = item_len * fld.size; ; field_item++, BIT--)
							if ((this.bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
								if (field_item == item) break;
								else BYTE += bytes;
						return true;
					case 5:
						if (field_items_total - 1 < item) throw new RuntimeException( "No room for item=" + item + ". The field_items_total =" + field_items_total );
						boolean multipack = fld.datatypes != null;
						if (length < 0 &&
						    get_bits( bytes, BIT_S - fld.field_info_bits - (item + 1) * len_bits, len_bits ) == 0)
							return false;
						int item_type_fix = item_type;
						
						if (item < field_item)
						{
							field_item = 0;
							BIT        = BIT_S - fld.field_info_bits;
							BYTE       = BYTE_S;
							item_len   = (int) get_bits( bytes, BIT -= len_bits, len_bits );
							if (multipack && 0 < item_len)
								item_len = type_len( fld.datatypes[item_type = item_len - 1], BYTE );
						}
						
						
						if (multipack)
							for (;
							     field_item < item;
							     BYTE += item_len,
									     item_type = (int) get_bits( bytes, BIT -= len_bits, len_bits ),
									     item_len = 0 < item_type ? type_len( fld.datatypes[--item_type], BYTE ) : 0,
									     field_item++)
								;
						else
							for (;
							     field_item < item;
							     BYTE += item_len * fld.size,
									     item_len = (int) get_bits( bytes, BIT -= len_bits, len_bits ),
									     field_item++)
								;
						if (length < 0) return true;
						if (multipack && length < fld.datatypes[item_type_fix].packMinBytes) length = fld.datatypes[item_type_fix].packMinBytes;
						if (item_len == length)
						{
							Arrays.fill( bytes, BYTE_S, BYTE_S + item_len, (byte) 0 );
							return true;
						}
						
						boolean resize = false;
						if (item_len == 0)
							insert( field_bit, 0, length * fld.size );
						else
						{
							resize_bytes( multipack ? length - item_len : (length - item_len) * fld.size );
							resize = true;
						}
						
						set_bits( multipack ? item_type_fix + 1 : length, len_bits, this.bytes, BIT );
						
						item_len = length;
						item_type = item_type_fix;
						if (resize) return true;
						break;
					case 6:
						if (field_item_0 <= item && item < field_item_0 + field_items)
						{
							int _BYTE = BYTE;
							if (item < _field_item)
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
							
							
							for (; ; bit--, _field_item++)
								if ((bytes[bit >> 3] & 1 << (bit & 7)) != 0)
								{
									bit -= len_bits;
									if (_field_item == item)
									{
										field_item = _field_item;
										BYTE       = _BYTE;
										BIT        = bit;
										item_len   = (int) get_bits( bytes, bit, len_bits );
										return true;
									}
									
									_BYTE += (int) get_bits( bytes, bit, len_bits ) * fld.size;
								}
								else if (_field_item == item) { break; }
						}
						
						if (length < 0)
							return false;
						if (item < field_item_0 || field_items == 0)
						{
							BIT  = BIT_S - fld.field_info_bits;
							BYTE = BYTE_S;
							int ins_items = field_items == 0 ? 1 : field_item_0 - item;
							insert( field_bit, ins_items + len_bits, length * fld.size );
							BIT = BIT_S - fld.field_info_bits - 1;
							set_bits( field_item_0 = item, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits );
							set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
						}
						else if (item < field_item_0 + field_items)
						{
							if (item < field_item)
							{
								field_item = field_item_0;
								BIT        = BIT_S - fld.field_info_bits - 1;
								BYTE       = BYTE_S;
							}
							else
							{
								BIT--;
								field_item++;
								BYTE += item_len * fld.size;
							}
							
							
							for (; field_item < item; BIT--, field_item++)
								if ((bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
								{
									BIT -= len_bits;
									item_len = (int) get_bits( bytes, BIT, len_bits );
									BYTE += item_len * fld.size;
								}
							
							assert( bytes[BIT >> 3] & 1 << (BIT & 7) ) == 0;
							insert( field_bit, len_bits, length * fld.size );
						}
						else
						{
							int ins_items = item - (field_item_0 + field_items) + 1;
							BIT  = BIT_E;
							BYTE = BYTE_E;
							insert( field_bit, ins_items + len_bits, length * fld.size );
							BIT  = BIT_E + len_bits;
							BYTE = BYTE_E - length * fld.size;
							set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
						}
						
						this.bytes[BIT >> 3] |= (byte) (1 << (BIT & 7));
						set_bits( item_len = length, len_bits, this.bytes, BIT -= len_bits );
						break;
					case 8:
					case 10:
						if (field_item_0 <= item && item < field_item_0 + field_items)
						{
							if (item < _field_item)
							{
								_field_item = field_item_0;
								bit         = BIT_S - fld.field_info_bits - 1;
							}
							else
							{
								bit--;
								_field_item++;
							}
							
							
							for (int bits_ = item_len * fld.size; ; bit--, _field_item++)
								if ((bytes[bit >> 3] & 1 << (bit & 7)) != 0)
								{
									bit -= bits_;
									if (_field_item == item)
									{
										field_item = item;
										BIT        = bit;
										return true;
									}
								}
								else if (_field_item == item) { break; }
						}
						
						if (length < 0)
							return false;
						int bits = item_len * fld.size;
						if (item < field_item_0 || field_items == 0)
						{
							BIT  = BIT_S - fld.field_info_bits;
							BYTE = BYTE_S;
							int ins_items = field_items == 0 ? 1 : field_item_0 - item;
							insert( field_bit, ins_items + bits, 0 );
							BIT = BIT_S - fld.field_info_bits - 1;
							set_bits( field_item_0 = item, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits );
							set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
						}
						else if (item < field_item_0 + field_items)
						{
							if (item < field_item)
							{
								field_item = field_item_0;
								BIT        = BIT_S - fld.field_info_bits - 1;
							}
							else
							{
								BIT--;
								field_item++;
							}
							
							
							for (; field_item < item; BIT--, field_item++)
								if ((bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
									BIT -= bits;
							assert( bytes[BIT >> 3] & 1 << (BIT & 7) ) == 0;
							insert( field_bit, bits, 0 );
						}
						else
						{
							int ins_items = item + 1 - (field_item_0 + field_items);
							BIT = BIT_E;
							insert( field_bit, ins_items + bits, 0 );
							BIT = BIT_E + bits;
							set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
						}
						
						this.bytes[BIT >> 3] |= (byte) (1 << (BIT & 7));
						BIT -= bits;
						break;
					case 11:
						if (item < field_items_total)
						{
							if (item < _field_item)
							{
								_field_item = -1;
								bit         = BIT_S - fld.field_info_bits;
							}
							
							int _items;
							do
							{
								bits = fld.size * (_items = (int) get_bits( bytes, bit -= len_bits, len_bits ));
								bit -= bits;
								_field_item++;
							} while (_field_item < item);
							
							if (0 < _items)
							{
								BIT        = bit;
								item_len   = _items;
								field_item = _field_item;
								return true;
							}
						}
						
						if (length < 0)
							return false;
						if (field_items_total - 1 < item)
							throw new RuntimeException( "No room for item=" + item + ". The field_items_total =" + field_items_total );
						if (item < field_item)
						{
							field_item = -1;
							BIT        = BIT_S - fld.field_info_bits;
						}
						
						while (field_item < item)
						{
							bits = (int) get_bits( bytes, BIT -= len_bits, len_bits ) * fld.size;
							BIT -= bits;
							field_item++;
						}
						
						if (get_bits( this.bytes, BIT, len_bits ) != 0) throw new RuntimeException( "Already allocated" );
						insert( field_bit, length * fld.size, 0 );
						set_bits( length, len_bits, this.bytes, BIT );
						BIT -= length * fld.size;
						item_len = length;
						break;
					case 12:
						if (field_item_0 <= item && item < field_item_0 + field_items)
						{
							if (item < _field_item)
							{
								_field_item = field_item_0;
								bit         = BIT_S - fld.field_info_bits - 1;
							}
							else
							{
								bit--;
								_field_item++;
							}
							
							
							for (; ; bit--, _field_item++)
								if ((bytes[bit >> 3] & 1 << (bit & 7)) != 0)
								{
									bit -= len_bits;
									if (_field_item == item)
									{
										field_item = _field_item;
										item_len   = (int) get_bits( bytes, bit, len_bits );
										BIT        = bit - item_len * fld.size;
										return true;
									}
									
									bit -= (int) get_bits( bytes, bit, len_bits ) * fld.size;
								}
								else if (_field_item == item) { break; }
						}
						
						if (length < 0)
							return false;
						if (item < field_item_0 || field_items == 0)
						{
							BIT = BIT_S - fld.field_info_bits;
							int ins_items = field_items == 0 ? 1 : field_item_0 - item;
							insert( field_bit, ins_items + len_bits + length * fld.size, 0 );
							BIT = BIT_S - fld.field_info_bits - 1;
							set_bits( field_item_0 = item, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits + fld.sparse_bits );
							set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
						}
						else if (item < field_item_0 + field_items)
						{
							if (item < field_item)
							{
								field_item = field_item_0;
								BIT        = BIT_S - fld.field_info_bits - 1;
							}
							else
							{
								BIT--;
								field_item++;
							}
							
							
							for (; field_item < item; BIT--, field_item++)
								if ((bytes[BIT >> 3] & 1 << (BIT & 7)) != 0)
								{
									BIT -= len_bits;
									item_len = (int) get_bits( bytes, BIT, len_bits );
									BIT -= item_len * fld.size;
								}
							
							assert( bytes[BIT >> 3] & 1 << (BIT & 7) ) == 0;
							insert( field_bit, len_bits + length * fld.size, 0 );
						}
						else
						{
							int ins_items = item - (field_item_0 + field_items) + 1;
							BIT = BIT_E;
							insert( field_bit, ins_items + len_bits + length * fld.size, 0 );
							BIT = BIT_E + length * fld.size + len_bits;
							set_bits( field_items += ins_items, fld.sparse_bits, bytes, BIT_S - fld.field_info_bits );
						}
						
						this.bytes[BIT >> 3] |= (byte) (1 << (BIT & 7));
						set_bits( item_len = length, len_bits, this.bytes, BIT -= len_bits );
						BIT -= length * fld.size;
						break;
				}
				
				field_item = item;
				return false;
			}
			
			
			private void resize_bytes( int diff ) {
				Cursor cur = this;
				while (cur.prev != null)
					cur = (Cursor) cur.prev;
				if (cur.pack_LAST_BYTE < 0) cur.set_pack_LASTS();
				byte[] new_bytes = new byte[cur.pack_LAST_BYTE + diff];
				System.arraycopy( bytes, 0, new_bytes, 0, BYTE );
				if (diff < 0)
					System.arraycopy( bytes, BYTE - diff, new_bytes, BYTE, cur.pack_LAST_BYTE - (BYTE - diff) );
				else
					System.arraycopy( bytes, BYTE, new_bytes, BYTE + diff, cur.pack_LAST_BYTE - BYTE );
				bytes = new_bytes;
				for (; ; )
				{
					if (-1 < cur.pack_LAST_BYTE) cur.pack_LAST_BYTE += diff;
					cur.BYTE_E += diff;
					cur.bytes = new_bytes;
					if (cur == this) return;
					if (-1 < cur.field_bit)
					{
						Meta.Field fld = cur.getField();
						cur.item_len += diff / fld.size;
						if (fld.datatypes == null)
						{
							int len_bits = -cur.getField().length;
							set_bits( cur.item_len, len_bits, bytes, cur.BIT );
						}
					}
					
					cur = (Cursor) cur.next_;
				}
			}
			
			
			public String decode() { return new String( bytes, BYTE, item_len, StandardCharsets.UTF_8 );}
			
			public char[] encode( String src, int index, char[] reuse ) {
				final int len  = src.length();
				int       size = len;
				
				if (reuse == null || reuse.length < len) reuse = src.toCharArray();
				else src.getChars( 0, len, reuse, 0 );
				
				for (int i = 0; ; i++)
					if (i == len)
					{
						if (index < 0)
							set_field( -index - 1, len );
						else
							set_item( index, len );
						
						for (i = 0; i < len; i++)
						     bytes[BYTE + i] = (byte) reuse[i];
						
						return reuse;
					}
					else if ('\u0080' <= reuse[i])
					{
						for (char ch; i < len; i++)
							if ((ch = reuse[i]) < 0x800) size += 0x7f - ch >>> 31;
							else
							{
								size += 2;
								if (MIN_SURROGATE <= ch && ch <= MAX_SURROGATE)
									if (Character.codePointAt( reuse, i ) == ch) size -= 3;
									else i++;
							}
						break;
					}
				
				if (index < 0)
					set_field( -index - 1, size );
				else
					set_item( index, size );
				
				
				for (int i = 0, ii = BYTE; i < len; )
				{
					final char ch = reuse[i++];
					
					if (ch < 0x80) bytes[ii++] = (byte) ch;
					else if (ch < 0x800)
					{
						bytes[ii++] = (byte) (0xc0 | ch >> 6);
						bytes[ii++] = (byte) (0x80 | ch & 0x3f);
					}
					else if (Character.MIN_SURROGATE <= ch && ch <= Character.MAX_SURROGATE)
					{
						final int uc = Character.codePointAt( reuse, i );
						if (uc == ch) bytes[ii++] = (byte) '?';
						else
						{
							bytes[ii++] = (byte) (0xf0 | uc >> 18);
							bytes[ii++] = (byte) (0x80 | uc >> 12 & 0x3f);
							bytes[ii++] = (byte) (0x80 | uc >> 6 & 0x3f);
							bytes[ii++] = (byte) (0x80 | uc & 0x3f);
							i++;
						}
					}
					else
					{
						bytes[ii++] = (byte) (0xe0 | ch >> 12);
						bytes[ii++] = (byte) (0x80 | ch >> 6 & 0x3f);
						bytes[ii++] = (byte) (0x80 | ch & 0x3f);
					}
				}
				return reuse;
			}
			
			private void insert( int fbit, int bits, int bytes ) {
				if (field_bit != fbit)
				{
					BIT      = BIT_S = BIT_E;
					BYTE     = BYTE_S = BYTE_E;
					item_len = 0;
				}
				
				if (pack_LAST_BYTE < 0) set_pack_LASTS();
				int old_bytes_len = pack_LAST_BYTE;
				int add_to_bits_bytes;
				int len_bits      = meta.BITS_lenINbytes_bits;
				if (BIT == pack_LAST_BIT && BYTE == old_bytes_len)
				{
					
					add_to_bits_bytes = len_bits == 0
					                    ? 0
					                    : bits2bytes( fbit + bits - (pack_LAST_BIT - origin * 8) + 1 );
					if (0 < add_to_bits_bytes || 0 < bytes)
					{
						resize_bytes( add_to_bits_bytes + bytes );
						if (0 < add_to_bits_bytes)
						{
							
							copy_bits( this.bytes, BIT,
									(old_bytes_len << 3) - BIT, this.bytes, BIT + (add_to_bits_bytes << 3) );
							set_0( this.bytes, BIT, add_to_bits_bytes << 3 );
						}
					}
					
					
				}
				else
				{
					
					
					int add_bits_bits = len_bits == 0
					                    ? 0
					                    : bits - (pack_LAST_BIT - (origin * 8 + pack_LAST_field_bit + 1));
					add_to_bits_bytes =
							len_bits == 0
							? 0
							: bits2bytes(
									add_bits_bits );
					if (0 < add_to_bits_bytes || 0 < bytes)
					{
						resize_bytes( add_to_bits_bytes + bytes );
						if (0 < bits)
						{
							
							copy_bits( this.bytes, BIT, (BYTE << 3) - BIT, this.bytes,
									BIT + (add_to_bits_bytes << 3) );
							
							
							copy_bits( this.bytes, pack_LAST_BIT,
									BIT - pack_LAST_BIT, this.bytes, pack_LAST_BIT + (add_to_bits_bytes << 3) - bits );
							
							
							if (0 < (add_to_bits_bytes << 3) - bits)
								set_0( this.bytes, pack_LAST_BIT, (add_to_bits_bytes << 3) - bits );
							set_0( this.bytes, BIT + (add_to_bits_bytes << 3) - bits, bits );
						}
					}
					else
					{
						copy_bits( this.bytes, pack_LAST_BIT,
								BIT - pack_LAST_BIT, this.bytes, pack_LAST_BIT - bits );
						set_0( this.bytes, BIT - bits, bits );
					}
				}
				
				if (0 < len_bits && 0 < add_to_bits_bytes
				)
				{
					int old_value = (int) get_bits( this.bytes, origin * 8 + meta.field_0_bit - len_bits, len_bits );
					set_bits( old_value + add_to_bits_bytes, len_bits, this.bytes, origin * 8 + meta.field_0_bit - len_bits );
				}
				
				int zazor_delta = (add_to_bits_bytes << 3) - bits;
				pack_LAST_BIT += zazor_delta;
				BIT += add_to_bits_bytes << 3;
				BYTE += add_to_bits_bytes;
				if (fbit == field_bit)
				{
					BIT_E += zazor_delta;
					BIT_S += add_to_bits_bytes << 3;
					BYTE_S += add_to_bits_bytes;
				}
				else
				{
					BIT_S  = BIT_E = BIT;
					BYTE_E = BYTE_S = BYTE;
					this.bytes[origin + (fbit >> 3)] |= (byte) (1 << (fbit & 7));
					if (pack_LAST_field_bit < fbit) pack_LAST_field_bit = fbit;
					field_bit = fbit;
				}
			}
		}
	}
	
	
	public abstract static class Channel {
		private static class Flow {
			
			final Pack.Meta.Field.CursorBase curs;
			Pack.Meta.Field.CursorBase cur;
			
			long Uvalue;
			char crc;
			
			protected STATE state = STATE.STANDBY;
			
			public Flow( int nested_max ) {
				cur    = curs = new Pack.Meta.Field.CursorBase( null, nested_max );
				state  = STATE.STANDBY;
				mode   = MODE.NONE;
				Uvalue = 0;
				crc    = 0;
			}
			
			public enum STATE
			
			{
				STANDBY, PACK_ID, VARINT, VARINT_BR, BYTES, BYTES_BR
			}
			
			MODE mode = MODE.NONE;
			
			enum MODE
			
			{
				OPTS_INFO, SET, NONE, CRC
			}
			
			Pack.Meta.Field.CursorBase next() {
				int fb = -1;
				for (boolean to_end = false; ; )
				{
start:
					{
						
						if (cur.field_bit == -1)
						{
							
							switch (cur.item_type)
							{
								case 0:
									cur.BYTE_S = cur.BYTE_E = cur.origin;
									if (0 < cur.meta._2)
									{
										cur.BYTE_E += cur.meta._2 * (cur.item_type = 2);
										state = STATE.VARINT;
										return cur;
									}
								case 2:
									if (0 < cur.meta._4)
									{
										cur.BYTE_E += cur.meta._4 * (cur.item_type = 4);
										state = STATE.VARINT;
										return cur;
									}
								case 4:
									if (0 < cur.meta._8)
									{
										cur.BYTE_E += cur.meta._8 * (cur.item_type = 8);
										state = STATE.VARINT;
										return cur;
									}
								case 8:
									if (cur.BYTE_S < (cur.BYTE_E = cur.origin + cur.meta.packMinBytes))
									{
										state         = STATE.BYTES;
										cur.item_type = 1;
										return cur;
									}
									mode = MODE.NONE;
								default:
									if (mode == MODE.NONE)
									{
										if (cur.meta.fields == null)
										{
											to_end = true;
											break;
										}
										int fix = cur.BYTE_S;
										cur.reset();
										if (fix < cur.BYTE_E)
										{
											cur.BYTE_S    = fix;
											state         = STATE.BYTES;
											mode          = MODE.OPTS_INFO;
											cur.item_type = 1;
											return cur;
										}
									}
									
									mode = MODE.NONE;
									cur.BIT_E = cur.BIT_S = (cur.BYTE_S = cur.BYTE_E) << 3;
									if ((fb = cur.next_field_bit()) < 0) to_end = true;
							}
						}
						else if ((fb = cur.next_field_bit()) < 0) to_end = true;
						
						
						do
						{
next_field:
							{
								if (!to_end)
									do
									{
										
										cur.field_bit = fb;
										Pack.Meta.Field fld = cur.getField();
										state      = fld.varint ? STATE.VARINT : STATE.BYTES;
										cur.BIT_S  = cur.BIT_E;
										cur.BYTE_S = cur.BYTE_E;
										
										if (fld.datatypes == null)
										{
											if (0 < fld.length)
												if (0 < fld.size)
													cur.BYTE_E += fld.const_dims_total * fld.length * (cur.item_type = fld.size);
												else
													cur.BIT_E += fld.const_dims_total * fld.length * fld.size;
											else cur.set_E( fld );
											if (cur.BYTE_S < cur.BYTE_E) return cur;
											
										}
										else
										{
											
											Pack.Meta.Field.CursorBase prev = cur;
											cur           = cur.next_;
											cur.origin    = prev.BYTE_E;
											cur.bytes     = prev.bytes;
											cur.field_bit = -1;
											cur.item_type = 0;
											
											
											prev.item_type = fld.const_dims_total;
											if (fld.var_dims != null)
												for (int i = 0, bit = prev.BIT_S; i < fld.var_dims.length; i++)
												     prev.item_type *= (int) get_bits( prev.bytes, bit -= fld.var_dims[i], fld.var_dims[i] );
											switch (fld.type)
											{
												case 1:
													cur.meta = fld.datatypes[0];
													break start;
												case 3:
													prev.BIT_E -= fld.field_info_bits;
													cur.meta = fld.datatypes[(int) get_bits( prev.bytes, prev.BIT_E, -fld.length )];
													break start;
												case 5:
													prev.BIT_E -= fld.field_info_bits;
													for (int item_type, len_bits = -fld.length; 0 < prev.item_type; prev.item_type--)
														if (0 < (item_type = (int) get_bits( prev.bytes, prev.BIT_E -= len_bits, len_bits )))
														{
															cur.meta = fld.datatypes[item_type - 1];
															break start;
														}
											}
										}
										
										
									}
									while (-1 < (fb = cur.next_field_bit()));
								else
									to_end = false;
								
								
								for (Pack.Meta.Field.CursorBase prev = cur.prev; prev != null; )
								{
									if (1 < prev.item_type)
									{
										Pack.Meta.Field fld = prev.getField();
										for (int item_type, len_bits = -fld.length; 1 < prev.item_type--; )
										{
											if (fld.type == 5)
												if (0 < (item_type = (int) get_bits( prev.bytes, prev.BIT_E -= len_bits, len_bits ))) cur.meta = fld.datatypes[item_type - 1];
												else continue;
											cur.origin    = cur.BYTE_E;
											cur.field_bit = -1;
											cur.item_type = 0;
											break start;
										}
									}
									
									prev.BYTE_E = cur.BYTE_E;
									cur         = prev;
									if (-1 < (fb = cur.next_field_bit())) break next_field;
									prev = cur.prev;
								}
								
								return null;
							}
						}
						while (true);
					}
				}
			}
			
			
		}
		
		public abstract static class Transmitter extends java.io.InputStream {
			final Flow flow;
			final int  id_bytes;
			
			protected abstract Pack pullSendingPack();
			
			public Transmitter( int nested_max, int id_bytes ) {
				flow          = new Flow( nested_max );
				this.id_bytes = id_bytes;
			}
			
			protected void failure( String reason ) { System.out.println( "failure: " + reason ); }
			
			@Override public int read()             {
				                                        final byte[] bytes = new byte[1];
				                                        return 0 < read( bytes, 0, 1 ) ? bytes[0] & 0xFF : -1;
			                                        }
			
			@Override public int read( byte[] dst, int BYTE, int bytes ) {
				final int fix = BYTE;
				for (Pack.Meta.Field.CursorBase cur = flow.cur; 0 < bytes--; BYTE++)
				{
					switch (flow.state)
					{
						case STANDBY:
							Pack pack = pullSendingPack();
							if (pack == null) return BYTE - fix;
							cur.wrap( pack );
							cur.field_bit = 8 * (id_bytes - 1);
							flow.state = Flow.STATE.PACK_ID;
							flow.Uvalue = pack.meta.id;
						
						case PACK_ID:
							dst[BYTE] = (byte) (0xFF & flow.Uvalue >> cur.field_bit);
							if (-1 < (cur.field_bit -= 8)) continue;
							cur.field_bit = -1;
							flow.mode = Flow.MODE.NONE;
							if (cur.meta.fields != null)
							{
								flow.state  = Flow.STATE.VARINT;
								flow.Uvalue = cur.type_len( cur.meta, cur.origin ) + 1 - (cur.origin + cur.meta.packMinBytes);
								cur.BIT_E   = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
								continue;
							}
							
							flow.Uvalue = 0;
							cur.BIT_E = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
							break;
						case BYTES:
							dst[BYTE] = cur.bytes[cur.BYTE_S++];
							if (cur.BYTE_S < cur.BYTE_E) continue;
							break;
						case VARINT:
							if ((flow.Uvalue & ~0x7FL) != 0)
							{
								dst[BYTE] = (byte) (flow.Uvalue & 0x7F | 0x80);
								flow.Uvalue >>= 7;
								continue;
							}
							
							dst[BYTE] = (byte) flow.Uvalue;
							if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E)
							{
								flow.Uvalue = get_bytes( cur.bytes, cur.BYTE_S, cur.item_type );
								continue;
							}
							
							break;
					}
					
					if ((cur = flow.next()) != null)
					{
						if (flow.state == Flow.STATE.VARINT)
							flow.Uvalue = get_bytes( cur.bytes, cur.BYTE_S, cur.item_type );
						continue;
					}
					
					flow.state = Flow.STATE.STANDBY;
					flow.mode  = Flow.MODE.NONE;
					cur        = flow.curs;
					do
					{
						cur.meta  = null;
						cur.bytes = null;
					} while ((cur = cur.next_) != null);
					
					cur = flow.curs;
				}
				
				return BYTE - fix;
			}
			
			public abstract static class Advanced extends Transmitter {
				public Advanced( int nested_max, int id_bytes ) { super( nested_max, id_bytes ); }
				
				@Override public int read( byte[] dst, int BYTE, int bytes ) {
					int                        fix = BYTE;
					Pack.Meta.Field.CursorBase cur = flow.cur;
					for (int t; 0 < bytes--; t = flow.mode != Flow.MODE.CRC ? flow.crc = crc16( dst[BYTE], flow.crc ) : 0, BYTE++)
					{
						switch (flow.state)
						{
							case STANDBY:
								Pack pack = pullSendingPack();
								if (pack == null) return BYTE - fix;
								cur.wrap( pack );
								cur.field_bit = 8 * (id_bytes - 1);
								flow.state = Flow.STATE.PACK_ID;
								flow.Uvalue = pack.meta.id;
								flow.crc = 0;
								dst[BYTE] = BR;
								continue;
							case PACK_ID:
								dst[BYTE] = (byte) (0xFF & flow.Uvalue >> cur.field_bit);
								if (-1 < (cur.field_bit -= 8)) continue;
								cur.field_bit = -1;
								flow.mode = Flow.MODE.NONE;
								if (cur.meta.fields != null)
								{
									flow.state  = Flow.STATE.VARINT;
									flow.Uvalue = cur.type_len( cur.meta, cur.origin ) + 1 - (cur.origin + cur.meta.packMinBytes);
									cur.BIT_E   = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
									continue;
								}
								
								flow.Uvalue = 0;
								cur.BIT_E = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
								break;
							case BYTES:
								if (flow.mode == Flow.MODE.CRC)
								{
									switch (cur.item_type)
									{
										case 4:
											cur.item_type = (dst[BYTE] = (byte) (flow.crc >> 8)) == BR ? 3 : 2;
											continue;
										case 3:
											dst[BYTE] = BR;
											cur.item_type = 2;
											continue;
										case 2:
											if ((dst[BYTE] = (byte) (flow.crc & 0xFF)) != BR) break;
											cur.item_type = 1;
											continue;
										case 1:
											dst[BYTE] = BR;
											break;
									}
									
									flow.state = Flow.STATE.STANDBY;
									flow.mode  = Flow.MODE.NONE;
									cur        = flow.curs;
									do
									{
										cur.meta  = null;
										cur.bytes = null;
									} while ((cur = cur.next_) != null);
									
									cur = flow.curs;
									continue;
								}
								
								if ((flow.Uvalue = dst[BYTE] = cur.bytes[cur.BYTE_S++]) == BR)
								{
									flow.state = Flow.STATE.BYTES_BR;
									continue;
								}
							
							case BYTES_BR:
								flow.state = Flow.STATE.BYTES;
								dst[BYTE] = (byte) flow.Uvalue;
								if (cur.BYTE_S < cur.BYTE_E) continue;
								break;
							case VARINT:
								if ((flow.Uvalue & ~0x7FL) != 0)
								{
									dst[BYTE] = (byte) (flow.Uvalue & 0x7F | 0x80);
									flow.Uvalue >>= 7;
									continue;
								}
								
								
								if (flow.Uvalue == BR)
								{
									flow.state = Flow.STATE.VARINT_BR;
									dst[BYTE]  = BR;
									continue;
								}
							
							
							case VARINT_BR:
								flow.state = Flow.STATE.VARINT;
								dst[BYTE] = (byte) flow.Uvalue;
								if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E)
								{
									flow.Uvalue = get_bytes( cur.bytes, cur.BYTE_S, cur.item_type );
									continue;
								}
								
								break;
						}
						
						if ((cur = flow.next()) != null)
						{
							if (flow.state == Flow.STATE.VARINT)
								flow.Uvalue = get_bytes( cur.bytes, cur.BYTE_S, cur.item_type );
							continue;
						}
						
						cur           = flow.curs;
						flow.state    = Flow.STATE.BYTES;
						flow.mode     = Flow.MODE.CRC;
						cur.item_type = 4;
					}
					
					return BYTE - fix;
				}
			}
		}
		
		public abstract static class Receiver extends java.io.OutputStream {
			int bits = 0;
			volatile int time;
			
			final Flow flow;
			final int  id_bytes;
			
			protected void failure( String reason ) { System.out.println( "failure: " + reason ); }
			
			public Receiver( int nested_max, int id_bytes ) {
				flow          = new Flow( nested_max );
				this.id_bytes = id_bytes;
			}
			
			protected abstract Pack.Meta dispatch( int id, Pack pack );
			
			@Override public void write( int b ) {
				byte[] bytes = new byte[1];
				bytes[0] = (byte) (b & 0xFF);
				write( bytes, 0, 1 );
			}
			
			@Override public void write( byte[] src, int BYTE, int bytes ) {
				time = ~0;
				if (time == 0)
				{
					time       = ~0;
					flow.state = Flow.STATE.STANDBY;
					failure( "Receive timeout" );
				}
				
				for (Pack.Meta.Field.CursorBase cur = flow.cur; 0 < bytes--; BYTE++)
				{
					switch (flow.state)
					{
						case STANDBY:
							flow.Uvalue = 0;
							bits = 0;
							flow.state = Flow.STATE.PACK_ID;
						case PACK_ID:
							flow.Uvalue = flow.Uvalue << 8 | src[BYTE];
							if (++bits < id_bytes) continue;
							flow.mode = Flow.MODE.NONE;
							Pack.Meta meta = dispatch( (int) flow.Uvalue, null );
							if (meta == null)
							{
								failure( "Unrecognized package ID = " + flow.Uvalue );
								flow.state = Flow.STATE.STANDBY;
								continue;
							}
							
							cur.meta = meta;
							flow.Uvalue = 0;
							bits = 0;
							cur.field_bit = -1;
							cur.BIT_E = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
							if (meta.fields != null)
							{
								flow.state = Flow.STATE.VARINT;
								flow.mode  = Flow.MODE.OPTS_INFO;
								continue;
							}
							else
							{
								cur.bytes = new byte[meta.packMinBytes];
								cur.reset();
							}
							
							break;
						case BYTES:
							cur.bytes[cur.BYTE_S++] = src[BYTE];
							if (cur.BYTE_S < cur.BYTE_E) continue;
							break;
						case VARINT:
							flow.Uvalue |= (src[BYTE] & 0x7FL) << bits;
							bits += 7;
							if ((src[BYTE] & 0x80) != 0) continue;
							bits = 0;
							if (flow.mode == Flow.MODE.OPTS_INFO)
							{
								cur.bytes   = new byte[cur.meta.packMinBytes + (int) flow.Uvalue];
								flow.Uvalue = 0;
								flow.mode   = Flow.MODE.NONE;
								break;
							}
							
							set_bytes( flow.Uvalue, cur.item_type, cur.bytes, cur.BYTE_S );
							flow.Uvalue = 0;
							if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E) continue;
							break;
					}
					
					if ((cur = flow.next()) != null) continue;
					cur = flow.curs;
					dispatch( cur.meta.id, cur.unwrap() );
					cur = flow.curs;
					do
					{
						cur.meta  = null;
						cur.bytes = null;
					} while ((cur = cur.next_) != null);
					
					cur        = flow.curs;
					flow.state = Flow.STATE.STANDBY;
					flow.mode  = Flow.MODE.NONE;
				}
			}
			
			public abstract static class Advanced extends Receiver {
				public Advanced( int nested_max, int id_bytes ) { super( nested_max, id_bytes ); }
				
				int bits = 0;
				
				
				@Override public void write( byte[] src, int BYTE, int bytes ) {
					time = ~0;
					if (time == 0)
					{
						time       = ~0;
						flow.state = Flow.STATE.STANDBY;
						failure( "Receive timeout" );
					}
					
					Pack.Meta.Field.CursorBase cur = flow.cur;
					for (int t; 0 < bytes--; t = flow.mode != Flow.MODE.CRC ? flow.crc = crc16( src[BYTE], flow.crc ) : 0, BYTE++)
					{
						switch (flow.state)
						{
							case STANDBY:
								flow.crc = 0;
								flow.Uvalue = 0;
								bits = 0;
								if (src[BYTE] == BR) flow.state = Flow.STATE.PACK_ID;
								continue;
							case PACK_ID:
								if (src[BYTE] == BR)
								{
									failure( " After BR expect helper ID but got +BR" );
									flow.state = Flow.STATE.STANDBY;
									continue;
								}
								
								flow.Uvalue = flow.Uvalue << 8 | src[BYTE];
								if (++bits < id_bytes) continue;
								flow.mode = Flow.MODE.NONE;
								Pack.Meta meta = dispatch( (int) flow.Uvalue, null );
								if (meta == null)
								{
									failure( "Unrecognized package ID = " + flow.Uvalue );
									flow.state = Flow.STATE.STANDBY;
									continue;
								}
								
								cur.meta = meta;
								flow.Uvalue = 0;
								bits = 0;
								cur.field_bit = -1;
								cur.BIT_E = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
								if (meta.fields != null)
								{
									flow.state = Flow.STATE.VARINT;
									flow.mode  = Flow.MODE.OPTS_INFO;
									continue;
								}
								else
								{
									cur.bytes = new byte[meta.packMinBytes];
									cur.reset();
								}
								
								break;
							case BYTES:
								if (src[BYTE] == BR)
								{
									flow.state = Flow.STATE.BYTES_BR;
									continue;
								}
							
							case BYTES_BR:
								if (flow.state == Flow.STATE.BYTES_BR)
								{
									if (src[BYTE] != BR)
									{
										failure( "waiting for second BR but got " + src[BYTE] );
										flow.state = Flow.STATE.STANDBY;
										continue;
									}
									
									flow.state = Flow.STATE.BYTES;
								}
								
								if (flow.mode == Flow.MODE.CRC)
									switch (cur.item_type)
									{
										case 2:
											flow.Uvalue = src[BYTE] << 8;
											cur.item_type = 1;
											continue;
										case 1:
											if ((flow.Uvalue | src[BYTE]) == flow.crc)
												dispatch( cur.meta.id, cur.unwrap() );
											else failure( "CRC error" );
											cur = flow.curs;
											do
											{
												cur.meta  = null;
												cur.bytes = null;
											} while ((cur = cur.next_) != null);
											
											flow.state = Flow.STATE.STANDBY;
											flow.mode = Flow.MODE.NONE;
											continue;
									}
								
								cur.bytes[cur.BYTE_S++] = src[BYTE];
								if (cur.BYTE_S < cur.BYTE_E) continue;
								break;
							case VARINT:
								if (src[BYTE] == BR)
								{
									flow.state = Flow.STATE.VARINT_BR;
									continue;
								}
							
							case VARINT_BR:
								if (flow.state == Flow.STATE.VARINT_BR)
								{
									if (src[BYTE] != BR)
									{
										failure( "waiting for second BR but got " + src[BYTE] );
										flow.state = Flow.STATE.STANDBY;
										continue;
									}
									
									flow.state = Flow.STATE.VARINT;
								}
								
								flow.Uvalue |= (src[BYTE] & 0x7FL) << bits;
								bits += 7;
								if ((src[BYTE] & 0x80) != 0) continue;
								bits = 0;
								if (flow.mode == Flow.MODE.OPTS_INFO)
								{
									cur.bytes   = new byte[cur.meta.packMinBytes + (int) flow.Uvalue];
									flow.Uvalue = 0;
									flow.mode   = Flow.MODE.NONE;
									break;
								}
								
								set_bytes( flow.Uvalue, cur.item_type, cur.bytes, cur.BYTE_S );
								flow.Uvalue = 0;
								if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E) continue;
								break;
						}
						
						if ((cur = flow.next()) != null) continue;
						cur           = flow.curs;
						flow.state    = Flow.STATE.BYTES;
						flow.mode     = Flow.MODE.CRC;
						cur.item_type = 2;
					}
				}
			}
		}
		
		private static final char[] tab = {0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548, 53677, 57806, 61935};
		
		
		private static char crc16( byte data, char crc ) {
			int UBYTE = data & 0xFF;
			crc = (char) (tab[(crc >> 12 ^ UBYTE >> 4) & 0x0F] ^ crc << 4);
			return (char) (tab[(crc >> 12 ^ UBYTE & 0x0F) & 0x0F] ^ crc << 4);
		}
		
		private static final byte BR = 0x55;
		
	}
	
	
	public static byte[] bytes( int... ints ) {
		byte[] bytes = new byte[ints.length];
		for (int i = ints.length - 1; -1 < i; i--) bytes[i] = (byte) ints[i];
		return bytes;
	}
	
	public static char[] chars( int... ints ) {
		char[] chars = new char[ints.length];
		for (int i = ints.length - 1; -1 < i; i--) chars[i] = (char) ints[i];
		return chars;
	}
	
	public static short[] shorts( int... ints ) {
		short[] shorts = new short[ints.length];
		for (int i = ints.length - 1; -1 < i; i--) shorts[i] = (short) ints[i];
		return shorts;
	}
}