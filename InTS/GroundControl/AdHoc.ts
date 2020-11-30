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
'use strict';

export namespace org.unirail.AdHoc {
	const Ol           = [0, 1, 3, 7, 15, 31, 63, 127, 255];
	const lO           = [0, 128, 192, 224, 240, 248, 252, 254, 255];
	const MAX_INT      = 2 ** 53 - 1;
	export const JS_HI = 0x1_0000_0000;
	
	export function get_bytes(src: DataView, byte: number, bytes: number): number {
		switch (bytes) {
			case 1:
				return src.getUint8(byte);
			case 2:
				return src.getUint16(byte, true);
			case 3:
				return (src.getUint8(byte) + src.getUint16(byte + 1, true)) << 8;
			case 4:
				return src.getUint32(byte, true);
			case 5:
				return src.getUint32(byte, true) + src.getUint8(byte + 4) * JS_HI;
			case 6:
				return src.getUint32(byte, true) + src.getUint16(byte + 4, true) * JS_HI;
			case 7:
				return src.getUint32(byte, true) + (src.getUint8(byte + 4) + src.getUint16(byte + 5, true)) * JS_HI;
			case 8:
				return src.getUint32(byte, true) + src.getUint32(byte + 4, true) * JS_HI;
		}
		return 0;
	}
	
	export function set_bytes(src: number, bytes: number, dst: DataView, byte: number) {
		switch (bytes) {
			case 1:
				dst.setUint8(byte, src);
				break;
			case 2:
				dst.setUint16(byte, src, true);
				break;
			case 3:
				dst.setUint8(byte, src);
				dst.setUint16(byte >>> 8, src, true);
				break;
			case 4:
				dst.setUint32(byte, src, true);
				break;
			case 5:
				dst.setUint32(byte, src % JS_HI, true);
				dst.setUint8(byte + 4, src / JS_HI);
			case 6:
				dst.setUint32(byte, src % JS_HI, true);
				dst.setUint16(byte + 4, src / JS_HI, true);
			case 7:
				dst.setUint32(byte, src % JS_HI, true);
				dst.setUint8(byte + 4, src / JS_HI);
				dst.setUint16(byte + 5, src / JS_HI, true);
			case 8:
				dst.setUint32(byte, src % JS_HI, true);
				dst.setUint32(byte + 4, src / JS_HI, true);
		}
	}
	
	export function ArrayEquals(a: ArrayBuffer, a_byte: number, b: ArrayBuffer, b_byte: number, bytes: number): number {
		if (bytes < 1 || Math.min(a.byteLength - a_byte, b.byteLength - b_byte, bytes) !== bytes) return -1;
		for (let i = 0, A = new Uint8Array(a, a_byte, bytes), B = new Uint8Array(b, b_byte, bytes); i < bytes; i++) if (A[i] !== B[i]) return i;
		return 0;
	}
	
	export function copy_bytes(src: ArrayBuffer, src_byte: number, dst: ArrayBuffer, dst_byte: number, bytes: number): ArrayBuffer {
		bytes = Math.min(src.byteLength - src_byte, dst.byteLength - dst_byte, bytes);
		
		if (0 < bytes) new Uint8Array(dst, dst_byte, bytes).set(new Uint8Array(src, src_byte, bytes));
		return dst;
	}
	
	export function set_0(dst: DataView, bit: number, bits: number) {
		let dst_byte = bit >> 3;
		bit &= 7;
		if (8 < bit + bits) {
			if (0 < bit) {
				dst.setUint8(dst_byte, dst.getUint8(dst_byte) & Ol[bit]);
				if ((bits -= 8 - bit) === 0) return;
				dst_byte++;
			}
			
			if (0 < (bits & 7)) dst.setUint8(dst_byte + (bits >> 3), dst.getUint8(dst_byte + (bits >> 3)) & lO[8 - (bits & 7)]);
			if ((bits >>= 3) === 0) return;
			for (let i = dst_byte + bits; dst_byte <= --i;) dst.setUint8(i, 0);
		}
		else {
			dst.setUint8(dst_byte, dst.getUint8(dst_byte) & (Ol[bit] | lO[8 - (bit + bits)]));
		}
	}
	
	export function get_bits(src: DataView, bit: number, bits: number): number {
		if (32 < bits) throw new Error('bits hav to be lower then 32');
		
		let src_byte = bit >> 3;
		bit &= 7;
		
		if (bit + bits < 9) return (src.getUint8(src_byte) >> bit) & Ol[bits];
		let dst = 0;
		for (let i = 0, last = ((bit + bits) >> 3) << 3; i < last; i += 8) dst |= src.getUint8(src_byte++) << i;
		
		dst >>>= bit;
		bit = (bit + bits) & 7;
		if (0 < bit) dst |= (src.getUint8(src_byte) & Ol[bit]) << (bits - bit);
		return dst;
	}
	
	export function set_bits(src: number, bits: number, dst: DataView, bit: number) {
		if (32 < bits) throw new Error('bits hav to be lower then 32');
		let dst_byte = bit >> 3;
		bit &= 7;
		if (8 < bit + bits) {
			if (0 < bit) {
				dst.setUint8(dst_byte, (dst.getUint8(dst_byte) & Ol[bit]) | ((src & Ol[8 - bit]) << bit));
				dst_byte++;
				src >>>= 8 - bit;
				bits -= 8 - bit;
			}
			
			for (let BYTE = 0, bytes = bits >> 3; BYTE < bytes; BYTE++, src >>>= 8) dst.setUint8(dst_byte++, src);
			
			if (0 < (bits &= 7)) dst.setUint8(dst_byte, (dst.getUint8(dst_byte) & lO[8 - bits]) | (src & Ol[bits]));
		}
		else {
			dst.setUint8(dst_byte, (dst.getUint8(dst_byte) & (Ol[bit] | lO[8 - bit - bits])) | ((src & Ol[bits]) << bit));
		}
	}
	
	export function copy_bits(src: DataView, src_bit: number, bits: number, dst: DataView, dst_bit: number) {
		if (bits === 0 || (src === dst && src_bit === dst_bit)) return;
		let count = bits >>> 5;
		bits &= 0x1f;
		if (src === dst && src_bit < dst_bit) {
			src_bit += count * 32;
			dst_bit += count * 32;
			if (0 < bits) set_bits(get_bits(src, src_bit, bits), bits, dst, dst_bit);
			for (; 0 < count--; src_bit -= 32, dst_bit -= 32, set_bits(get_bits(src, src_bit, 32), 32, dst, dst_bit)) ;
		}
		else {
			for (; 0 < count; set_bits(get_bits(src, src_bit, 32), 32, dst, dst_bit), src_bit += 32, dst_bit += 32, count--) ;
			if (0 < bits) set_bits(get_bits(src, src_bit, bits), bits, dst, dst_bit);
		}
	}
	
	export function first_1(bytes: DataView, bit: number, bits: number, existence: boolean): number {
		if (bits < 1) return -1;
		let _1BYTE = bit >> 3;
		let v      = bytes.getUint8(_1BYTE);
		bit &= 7;
		if (bits === 1) return (v & (1 << bit)) === 0 ? -1 : 0;
		let add = 0;
		sBreak: {
			if (0 < bit) {
				if (0 < (v >>= bit)) {
					if (bit + bits < 8 && (v & Ol[bits]) === 0) return -1;
					break sBreak;
				}
				
				if (bit + bits < 8) return -1;
				bits -= add = 8 - bit;
				_1BYTE++;
			}
			else {
				if (bits < 9)
					if (v === 0 || (v & Ol[bits]) === 0) return -1;
					else break sBreak;
			}
			
			let last = _1BYTE + (bits >> 3);
			for (let BYTE = _1BYTE; BYTE < last; BYTE++)
				if (0 < (v = bytes.getUint8(BYTE))) {
					add += (BYTE - _1BYTE) << 3;
					break sBreak;
				}
			
			if ((bits &= 7) === 0 || (v = bytes.getUint8(last) & Ol[bits]) === 0) return -1;
			add += (last - _1BYTE) << 3;
		}
		
		if (existence) return MAX_INT;
		for (let i = 0; ; i++) if (((v >> i) & 1) === 1) return add + i;
	}
	
	export function bits2bytes(bits: number): number {
		return bits < 1 ? 0 : 1 + ((bits - 1) >> 3);
	}
	
	export class Pack {
		public bytes: ArrayBuffer = null!;
		
		constructor(public readonly meta: Pack.Meta) {}
	}
	
	export namespace Pack {
		import CursorBase = org.unirail.AdHoc.Pack.Meta.Field.CursorBase;
		
		export class Meta {
			public readonly fields: Array<Meta.Field>;
			
			constructor(public readonly id: number, public readonly _2 = 0, public readonly _4 = 0, public readonly _8 = 0, public readonly packMinBytes = 0, public readonly nesting_max = 1, public readonly field_0_bit = 0, public readonly BITS_lenINbytes_bits = 0, fields = 0) {
				if (fields) this.fields = new Array<Meta.Field>(fields);
			}
		}
		
		export namespace Meta {
			export class Field {
				public readonly var_dims: number[];
				public readonly datatypes: Meta[];
				
				constructor(
					public type                      = 0,
					public varint: boolean,
					public readonly length           = 0,
					public readonly size             = 0,
					public readonly const_dims_total = 0,
					public readonly field_info_bits  = 0,
					public readonly sparse_bits      = 0,
					datatypes?: Meta[],
					...var_dims: number[]
				) {
					this.var_dims  = var_dims;
					this.datatypes = datatypes!;
				}
			}
			
			export namespace Field {
				export class CursorBase {
					public bytes: DataView;
					public meta: Meta;
					public origin = 0;
					public shift  = 0;
					
					public last(): boolean {
						return this.next_ == null;
					}
					
					next_: CursorBase | null;
					
					prev: CursorBase | null;
					
					public constructor(nested_max?: number, prev?: CursorBase) {
						if (nested_max !== undefined) {
							this.prev  = prev!;
							this.next_ = 1 < nested_max ? new CursorBase(nested_max - 1, this) : <CursorBase>null!;
						}
					}
					
					public BIT_E = 0;
					
					public BIT_S  = -1;
					public BYTE_E = -1;
					
					public BYTE_S = -1;
					
					public field_bit = 0;
					
					public wrap(src: Meta | Pack) {
						if (src instanceof Meta) {
							this.meta  = src;
							this.bytes = new DataView(new ArrayBuffer(src.packMinBytes));
						}
						else {
							this.meta  = src.meta;
							this.bytes = src.bytes ? new DataView(src.bytes) : null!;
							src.bytes  = null!;
						}
						this.origin = 0;
						this.reset();
					}
					
					public unwrap(): Pack | undefined {
						if (!this.meta) return undefined;
						
						let dst   = new Pack(this.meta);
						dst.bytes = this.bytes.buffer;
						
						this.meta  = <Meta>null!;
						this.bytes = <DataView>null!;
						
						return dst;
					}
					
					public item_type = 0;
					
					public next_field_bit(): number {
						if (this.field_bit - this.meta.field_0_bit < this.meta.fields.length - 1) {
							const bit = this.origin * 8 + (this.field_bit < 0 ? this.meta.field_0_bit : this.field_bit + 1);
							const i   = first_1(this.bytes, bit, Math.min(this.BIT_E - bit, this.meta.fields.length), false);
							return i < 0 ? -1 : i + bit - this.origin * 8;
						}
						
						return -1;
					}
					
					public getField(): Field {
						return this.meta.fields[this.field_bit - this.meta.field_0_bit];
					}
					
					public reset(): boolean {
						this.field_bit = -1;
						const len_bits = this.meta.BITS_lenINbytes_bits;
						this.BYTE_E    = this.BYTE_S = this.origin + this.meta.packMinBytes + (len_bits == 0 ? 0 : get_bits(this.bytes, this.origin * 8 + this.meta.field_0_bit - len_bits, len_bits));
						this.BIT_E     = this.BIT_S = this.BYTE_E << 3;
						this.item_type = 0;
						return true;
					}
					
					type_len(TYPE: Meta, BYTE: number): number {
						if (TYPE.fields == null) return TYPE.packMinBytes;
						const bit_0    = BYTE * 8 + TYPE.field_0_bit;
						const len_bits = TYPE.BITS_lenINbytes_bits;
						
						const LAST_BYTE = BYTE + TYPE.packMinBytes + (len_bits == 0 ? 0 : get_bits(this.bytes, bit_0 - len_bits, len_bits));
						
						let fb = first_1(this.bytes, bit_0, Math.min(LAST_BYTE * 8 - bit_0, this.meta.fields.length), false);
						if (fb == -1) return LAST_BYTE - BYTE;
						fb += TYPE.field_0_bit;
						
						const _BIT_E     = this.BIT_E,
						      _BIT_S     = this.BIT_S,
						      _BYTE_S    = this.BYTE_S,
						      _BYTE_E    = this.BYTE_E,
						      _item_type = this.item_type,
						      _origin    = this.origin,
						      _shift     = this.shift,
						      _field_bit = this.field_bit;
						const _meta      = this.meta;
						this.meta        = TYPE;
						this.origin      = BYTE;
						this.BYTE_E      = this.BYTE_S = LAST_BYTE;
						this.BIT_E       = this.BIT_S = LAST_BYTE << 3;
						do {
							this.field_bit = fb;
							this.BIT_S     = this.BIT_E;
							this.BYTE_S    = this.BYTE_E;
							const fld      = this.getField();
							if (0 < fld.length)
								if (0 < fld.size) this.BYTE_E += fld.const_dims_total * fld.length * fld.size;
								else this.BIT_E += fld.const_dims_total * fld.length * fld.size;
							else this.set_E(fld);
						} while (-1 < (fb = this.next_field_bit()));
						
						const ret      = this.BYTE_E - BYTE;
						this.BIT_E     = _BIT_E;
						this.BIT_S     = _BIT_S;
						this.BYTE_S    = _BYTE_S;
						this.BYTE_E    = _BYTE_E;
						this.item_type = _item_type;
						this.origin    = _origin;
						this.shift     = _shift;
						this.field_bit = _field_bit;
						this.meta      = _meta;
						return ret;
					}
					
					set_E(fld: Field) {
						let bit = this.BIT_S;
						
						let count = fld.const_dims_total;
						if (fld.var_dims != null) for (let i = 0; i < fld.var_dims.length; i++) count *= get_bits(this.bytes, (bit -= fld.var_dims[i]), fld.var_dims[i]);
						
						switch (fld.type) {
							case 1:
								this.BIT_E -= fld.field_info_bits;
								this.BYTE_E += count * (fld.datatypes == null ? -fld.length * fld.size : this.type_len(fld.datatypes[(this.item_type = 0)], this.BYTE_S));
								break;
							case 3:
								this.BIT_E -= fld.field_info_bits;
								count *= get_bits(this.bytes, this.BIT_E, -fld.length);
								
								this.BYTE_E += fld.datatypes == null ? count * fld.size : this.type_len(fld.datatypes[(this.item_type = count)], this.BYTE_E);
								break;
							case 5:
								this.BIT_E -= fld.field_info_bits;
								if (fld.datatypes == null) {
									let all_arrays_sum = 0;
									while (0 < count--) all_arrays_sum += get_bits(this.bytes, (this.BIT_E -= -fld.length), -fld.length);
									this.BYTE_E += all_arrays_sum * fld.size;
								}
								else while (0 < count--) if (0 < (this.item_type = get_bits(this.bytes, (this.BIT_E -= -fld.length), -fld.length))) this.BYTE_E += this.type_len(fld.datatypes[--this.item_type], this.BYTE_E);
								
								break;
							case 7:
								this.BIT_E -= fld.field_info_bits + count * -fld.length * fld.size;
								break;
							case 9:
								this.BIT_E -= fld.field_info_bits;
								count *= get_bits(this.bytes, this.BIT_E, -fld.length);
								
								this.BIT_E -= count * fld.size;
								break;
							case 11:
								this.BIT_E -= fld.field_info_bits;
								while (0 < count--) {
									const bits = get_bits(this.bytes, (this.BIT_E -= -fld.length), -fld.length) * fld.size;
									this.BIT_E -= bits;
								}
								
								break;
							default:
								if ((count = get_bits(this.bytes, (this.BIT_E -= fld.field_info_bits), fld.sparse_bits)) == 0) return;
								bit = this.BIT_E;
								switch (fld.type) {
									case 2:
										this.BIT_E -= count;
										while (this.BIT_E < bit--) if ((this.bytes[bit >> 3] & (1 << (bit & 7))) == 0) count--;
										this.BYTE_E += -fld.length * count * fld.size;
										break;
									case 4:
										this.BIT_E -= count;
										count *= get_bits(this.bytes, bit + 2 * fld.sparse_bits, -fld.length);
										while (this.BIT_E < bit--) if ((this.bytes[bit >> 3] & (1 << (bit & 7))) == 0) count--;
										this.BYTE_E += count * fld.size;
										break;
									case 6:
										let all_arrays_sum = 0;
										while (0 < count--) if ((this.bytes[--bit >> 3] & (1 << (bit & 7))) != 0) all_arrays_sum += get_bits(this.bytes, (bit -= -fld.length), -fld.length);
										this.BIT_E = bit;
										this.BYTE_E += all_arrays_sum * fld.size;
										break;
									case 8:
										for (let bits = -fld.length * fld.size; 0 < count--;) if ((this.bytes[--bit >> 3] & (1 << (bit & 7))) != 0) bit -= bits;
										this.BIT_E = bit;
										break;
									case 10:
										for (let bits = get_bits(this.bytes, bit + 2 * fld.sparse_bits, -fld.length) * fld.size; 0 < count--;) if ((this.bytes[--bit >> 3] & (1 << (bit & 7))) != 0) bit -= bits;
										this.BIT_E = bit;
										break;
									case 12:
										while (0 < count--)
											if ((this.bytes[--bit >> 3] & (1 << (bit & 7))) != 0) {
												const len = get_bits(this.bytes, (bit -= -fld.length), -fld.length);
												bit -= len * fld.size;
											}
										
										this.BIT_E = bit;
										break;
									default:
										return;
								}
						}
					}
				}
			}
		}
		
		export class Cursor extends Meta.Field.CursorBase {
			private as_pack_ = new Cursor.View(this);
			
			public as_pack<T extends Object>(proto: T): T {
				Object.setPrototypeOf(this.as_pack_, proto);
				return <T>(<unknown>this.as_pack_);
			}
			
			private as_field_ = new Cursor.View(this);
			
			public as_field<T extends Object>(proto: T): T {
				Object.setPrototypeOf(this.as_field_, proto);
				return <T>(<unknown>this.as_field_);
			}
			
			private as_item_ = new Cursor.View(this);
			
			public as_item<T extends Object>(proto: T): T {
				Object.setPrototypeOf(this.as_item_, proto);
				return <T>(<unknown>this.as_item_);
			}
			
			public var_dims: number[];
			
			public BIT = -1;
			
			public BYTE = 0;
			
			private field_item = 0;
			
			private field_item_0       = 0;
			private field_items        = 0;
			private field_items_total  = 0;
			public item_len            = 0;
			public pack_LAST_BIT       = 0;
			public pack_LAST_BYTE      = 0;
			public pack_LAST_field_bit = 0;
			
			public constructor(prev: Cursor, nested_max: number, var_dim_len: number) {
				super();
				if (0 < var_dim_len) this.var_dims = new Array<number>(var_dim_len);
				this.prev  = prev;
				this.next_ = 1 < nested_max ? new Cursor(this, nested_max - 1, var_dim_len) : null;
			}
			
			protected set_pack_LASTS(): number {
				let fb = this.next_field_bit();
				if (fb < 0) {
					this.pack_LAST_BYTE      = this.BYTE_E;
					this.pack_LAST_BIT       = this.BIT_E;
					this.pack_LAST_field_bit = this.field_bit;
					return this.pack_LAST_BYTE;
				}
				
				const BIT_E_FX     = this.BIT_E,
				      BIT_S_FX     = this.BIT_S,
				      BYTE_S_FX    = this.BYTE_S,
				      BYTE_E_FX    = this.BYTE_E,
				      item_type_FX = this.item_type,
				      field_bit_FX = this.field_bit;
				do {
					this.BIT_S     = this.BIT_E;
					this.BYTE_S    = this.BYTE_E;
					this.field_bit = fb;
					const fld      = this.getField();
					if (0 < fld.length)
						if (0 < fld.size) this.BYTE_E += fld.const_dims_total * fld.length * fld.size;
						else this.BIT_E += fld.const_dims_total * fld.length * fld.size;
					else this.set_E(fld);
				} while (-1 < (fb = this.next_field_bit()));
				
				this.pack_LAST_BYTE      = this.BYTE_E;
				this.pack_LAST_BIT       = this.BIT_E;
				this.pack_LAST_field_bit = this.field_bit;
				this.BIT_E               = BIT_E_FX;
				this.BIT_S               = BIT_S_FX;
				this.BYTE_S              = BYTE_S_FX;
				this.BYTE_E              = BYTE_E_FX;
				this.item_type           = item_type_FX;
				this.field_bit           = field_bit_FX;
				return this.pack_LAST_BYTE;
			}
			
			public length(): number {
				return (-1 < this.pack_LAST_BYTE ? this.pack_LAST_BYTE : this.set_pack_LASTS()) - this.origin;
			}
			
			public next<T extends Object>(origin: number, proto: T): T {
				const next_  = this.next_!;
				next_.origin = origin;
				next_.bytes  = this.bytes;
				next_.meta   = this.getField().datatypes[this.item_type];
				next_.reset();
				return (<Cursor>next_).as_pack(proto);
			}
			
			public reset(): boolean {
				super.reset();
				if (this.meta.fields == null || this.next_field_bit() < 0) {
					this.pack_LAST_BYTE      = this.BYTE_E;
					this.pack_LAST_BIT       = this.BIT_E;
					this.pack_LAST_field_bit = this.meta.field_0_bit;
				}
				else this.pack_LAST_BYTE = this.pack_LAST_BIT = this.pack_LAST_field_bit = -1;
				this.BYTE         = this.BYTE_S;
				this.BIT          = this.BIT_S;
				this.item_len     = 0;
				this.field_item_0 = -1;
				this.field_item   = MAX_INT;
				this.field_items  = 0;
				return true;
			}
			
			public unwrap(): Pack {
				let ret = super.unwrap();
				for (let cur: CursorBase = this; (cur = cur.next_!); cur.bytes = null!, cur.meta = null!) ;
				return ret!;
			}
			
			public set_field(fbit: number, each_item_size: number, ...var_dims: number[]): boolean {
				if (this.field_bit === fbit) return true;
				if (each_item_size < 0 && (this.BIT_E <= this.origin * 8 + fbit || this.meta.fields.length <= fbit - this.meta.field_0_bit || (this.bytes[this.origin + (fbit >> 3)] & (1 << (fbit & 7))) == 0)) return false;
				
				let fld: Meta.Field;
				
				insert_field: {
					for (; ;) {
						let next = this.next_field_bit();
						if (fbit < next || next == -1) break;
						this.BIT_S     = this.BIT_E;
						this.BYTE_S    = this.BYTE_E;
						this.field_bit = next;
						fld            = this.getField();
						if (0 < fld.length)
							if (0 < fld.size) this.BYTE_E += fld.const_dims_total * fld.length * fld.size;
							else this.BIT_E += fld.const_dims_total * fld.length * fld.size;
						else this.set_E(fld);
						if (this.field_bit < fbit) continue;
						
						this.field_items_total = fld.const_dims_total;
						if (fld.var_dims != null) for (let i = 0, bit = this.BIT_S; i < fld.var_dims.length; i++) this.field_items_total *= this.var_dims[i] = get_bits(this.bytes, (bit -= fld.var_dims[i]), fld.var_dims[i]);
						
						break insert_field;
					}
					
					if (each_item_size < 0) return false;
					
					fld = this.meta.fields[fbit - this.meta.field_0_bit];
					
					if (fld.datatypes != null && this.next_ == null) return false;
					
					if (fld.type == 0) {
						if (0 < fld.size) {
							this.insert(fbit, 0, fld.const_dims_total * fld.length * fld.size);
							this.BYTE_E = (this.BYTE = this.BYTE_S) + fld.const_dims_total * (this.item_len = fld.length) * fld.size;
						}
						else {
							this.insert(fbit, fld.const_dims_total * fld.length * -fld.size, 0);
							this.BIT = this.BIT_E = this.BIT_S + fld.const_dims_total * (this.item_len = fld.length) * fld.size;
						}
						
						return true;
					}
					
					let total = fld.const_dims_total;
					if (fld.var_dims != null) for (let i = 0; i < fld.var_dims.length; i++) total *= this.var_dims[i] = var_dims[i];
					
					switch (fld.type) {
						case 1:
							this.insert(fbit, fld.field_info_bits, total * (fld.datatypes == null ? -fld.length * fld.size : Math.max(each_item_size, fld.datatypes[0].packMinBytes)));
							this.item_type = 0;
							break;
						case 2:
						case 6:
						case 8:
						case 12:
							this.insert(fbit, fld.field_info_bits, 0);
							break;
						case 3:
							const fix = this.item_type;
							this.insert(fbit, fld.field_info_bits, total * (fld.datatypes == null ? each_item_size * fld.size : (each_item_size = Math.max(each_item_size, fld.datatypes[fix].packMinBytes))));
							
							set_bits(fld.datatypes == null ? each_item_size : fix, -fld.length, this.bytes, this.BIT_S - fld.field_info_bits);
							this.item_type = fix;
							break;
						case 4:
							this.insert(fbit, fld.field_info_bits, 0);
							set_bits(each_item_size, -fld.length, this.bytes, this.BIT_S - fld.field_info_bits + 2 * fld.sparse_bits);
							break;
						case 5:
						case 11:
							this.insert(fbit, fld.field_info_bits + total * -fld.length, 0);
							break;
						case 7:
							this.insert(fbit, fld.field_info_bits + total * -fld.length * fld.size, 0);
							break;
						case 9:
							this.insert(fbit, fld.field_info_bits + total * each_item_size * fld.size, 0);
							set_bits(each_item_size, -fld.length, this.bytes, this.BIT_S - fld.field_info_bits);
							break;
						case 10:
							this.insert(fbit, fld.field_info_bits, 0);
							set_bits(each_item_size, -fld.length, this.bytes, this.BIT_S - fld.field_info_bits + 2 * fld.sparse_bits);
							break;
						default:
							break;
					}
					
					this.field_items_total = total;
					if (fld.var_dims != null) for (let i = 0, bit = this.BIT_S; i < var_dims.length; i++) set_bits(var_dims[i], fld.var_dims[i], this.bytes, (bit -= fld.var_dims[i]));
					
					if (0 < fld.length)
						if (0 < fld.size) this.BYTE_E += fld.const_dims_total * fld.length * fld.size;
						else this.BIT_E += fld.const_dims_total * fld.length * fld.size;
					else this.set_E(fld);
				}
				
				switch (fld.type) {
					case 0:
						this.BIT = this.BIT_E;
						break;
					case 1:
						this.item_len = fld.datatypes == null ? -fld.length : this.type_len(fld.datatypes[(this.item_type = 0)], this.BYTE_S);
						this.BIT      = this.BIT_E;
						break;
					case 3:
						this.item_len = get_bits(this.bytes, (this.BIT = this.BIT_S - fld.field_info_bits), -fld.length);
						if (fld.datatypes != null) this.item_len = this.type_len(fld.datatypes[(this.item_type = this.item_len)], this.BYTE_S);
						break;
					case 5:
					case 11:
						this.item_len     = 0;
						this.BIT          = this.BIT_S - fld.field_info_bits;
						this.field_item_0 = 0;
						this.field_item   = MAX_INT;
						this.field_items  = this.field_items_total;
						break;
					case 7:
						this.item_len     = -fld.length;
						this.field_item_0 = 0;
						this.BIT          = this.BIT_E;
						this.field_item   = (this.field_items = this.field_items_total) == 0 ? MAX_INT : 0;
						break;
					case 9:
						this.item_len     = get_bits(this.bytes, (this.BIT = this.BIT_S - fld.field_info_bits), -fld.length);
						this.field_item_0 = 0;
						this.BIT          = this.BIT_E;
						this.field_item   = (this.field_items = this.field_items_total) == 0 ? MAX_INT : 0;
						break;
					default:
						this.BIT = this.BIT_S - fld.field_info_bits;
						switch (fld.type) {
							case 2:
							case 6:
							case 8:
							case 12:
								this.item_len = -fld.length;
								this.BIT      = this.BIT_S - fld.field_info_bits;
								break;
							case 4:
							case 10:
								this.item_len = get_bits(this.bytes, (this.BIT = this.BIT_S - fld.field_info_bits) + 2 * fld.sparse_bits, -fld.length);
								break;
							default:
								break;
						}
						
						this.field_item   = MAX_INT;
						this.field_items  = get_bits(this.bytes, this.BIT, fld.sparse_bits);
						this.field_item_0 = get_bits(this.bytes, this.BIT + fld.sparse_bits, fld.sparse_bits);
						break;
				}
				
				this.BYTE = this.BYTE_S;
				return true;
			}
			
			public set_item(item: number, length: number): boolean {
				const fld = this.getField();
				if (this.field_item == item && fld.type != 5) return true;
				let bit         = this.BIT;
				let _field_item = this.field_item;
				const len_bits  = -fld.length;
				switch (fld.type) {
					default:
						return false;
					case 2:
					case 4:
						if (this.field_items == 0 || item < this.field_item_0 || this.field_item_0 + this.field_items <= item || (this.bytes[(bit = this.BIT_S - fld.field_info_bits - 1 - (item - this.field_item_0)) >> 3] & (1 << (bit & 7))) == 0) {
							if (length < 0) return false;
							if (item < this.field_item_0 || this.field_items == 0) {
								this.BIT        = this.BIT_S - fld.field_info_bits;
								this.BYTE       = this.BYTE_S;
								const ins_items = this.field_items == 0 ? 1 : this.field_item_0 - item;
								this.insert(this.field_bit, ins_items, this.item_len * fld.size);
								this.BIT = this.BIT_S - fld.field_info_bits - 1;
								set_bits((this.field_item_0 = item), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits + fld.sparse_bits);
								set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
							}
							else if (item < this.field_item_0 + this.field_items) {
								if (item < this.field_item) {
									this.field_item = this.field_item_0;
									this.BIT        = this.BIT_S - fld.field_info_bits - 1;
									this.BYTE       = this.BYTE_S;
								}
								else {
									this.BIT--;
									this.field_item++;
									this.BYTE += this.item_len * fld.size;
								}
								
								for (const bytes = this.item_len * fld.size; this.field_item < item; this.field_item++, this.BIT--) if ((this.bytes[this.BIT >> 3] & (1 << (this.BIT & 7))) != 0) this.BYTE += bytes;
								
								this.insert(this.field_bit, 0, this.item_len * fld.size);
							}
							else {
								const ins_items = item + 1 - (this.field_item_0 + this.field_items);
								this.BIT        = this.BIT_E;
								this.BYTE       = this.BYTE_E;
								this.insert(this.field_bit, ins_items, this.item_len * fld.size);
								
								this.BIT  = this.BIT_E;
								this.BYTE = this.BYTE_E - this.item_len * fld.size;
								set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
							}
							
							this.bytes[this.BIT >> 3] |= 1 << (this.BIT & 7);
							break;
						}
						
						if (item < this.field_item) {
							this.field_item = this.field_item_0;
							this.BIT        = this.BIT_S - fld.field_info_bits - 1;
							this.BYTE       = this.BYTE_S;
						}
						else {
							this.BIT--;
							this.field_item++;
							this.BYTE += this.item_len * fld.size;
						}
						
						for (const bytes = this.item_len * fld.size; ; this.field_item++, this.BIT--)
							if ((this.bytes[this.BIT >> 3] & (1 << (this.BIT & 7))) != 0)
								if (this.field_item == item) break;
								else this.BYTE += bytes;
						return true;
					case 5:
						if (this.field_items_total - 1 < item) throw new Error('No room for item=' + item + '. The field_items_total =' + this.field_items_total);
						const multipack = fld.datatypes != null;
						if (length < 0 && get_bits(this.bytes, this.BIT_S - fld.field_info_bits - (item + 1) * len_bits, len_bits) == 0) return false;
						const item_type_fix = this.item_type;
						
						if (item < this.field_item) {
							this.field_item = 0;
							this.BIT        = this.BIT_S - fld.field_info_bits;
							this.BYTE       = this.BYTE_S;
							this.item_len   = get_bits(this.bytes, (this.BIT -= len_bits), len_bits);
							if (multipack && 0 < this.item_len) this.item_len = this.type_len(fld.datatypes[(this.item_type = this.item_len - 1)], this.BYTE);
						}
						
						if (multipack) for (; this.field_item < item; this.BYTE += this.item_len, this.item_type = get_bits(this.bytes, (this.BIT -= len_bits), len_bits), this.item_len = 0 < this.item_type ? this.type_len(fld.datatypes[--this.item_type], this.BYTE) : 0, this.field_item++) ;
						else for (; this.field_item < item; this.BYTE += this.item_len * fld.size, this.item_len = get_bits(this.bytes, (this.BIT -= len_bits), len_bits), this.field_item++) ;
						if (length < 0) return true;
						if (multipack && length < fld.datatypes[item_type_fix].packMinBytes) length = fld.datatypes[item_type_fix].packMinBytes;
						if (this.item_len == length) {
							for (let i = this.BYTE_S, max = this.BYTE_S + this.item_len; i < max; i++) this.bytes.setInt8(i, 0);
							return true;
						}
						
						let resize = false;
						if (this.item_len == 0) this.insert(this.field_bit, 0, length * fld.size);
						else {
							this.resize_bytes(multipack ? length - this.item_len : (length - this.item_len) * fld.size);
							resize = true;
						}
						
						set_bits(multipack ? item_type_fix + 1 : length, len_bits, this.bytes, this.BIT);
						
						this.item_len  = length;
						this.item_type = item_type_fix;
						if (resize) return true;
						break;
					case 6:
						if (this.field_item_0 <= item && item < this.field_item_0 + this.field_items) {
							let _BYTE = this.BYTE;
							if (item < _field_item) {
								_field_item = this.field_item_0;
								bit         = this.BIT_S - fld.field_info_bits - 1;
								_BYTE       = this.BYTE_S;
							}
							else {
								bit--;
								_field_item++;
								_BYTE += this.item_len * fld.size;
							}
							
							for (; ; bit--, _field_item++)
								if ((this.bytes[bit >> 3] & (1 << (bit & 7))) != 0) {
									bit -= len_bits;
									if (_field_item == item) {
										this.field_item = _field_item;
										this.BYTE       = _BYTE;
										this.BIT        = bit;
										this.item_len   = get_bits(this.bytes, bit, len_bits);
										return true;
									}
									
									_BYTE += get_bits(this.bytes, bit, len_bits) * fld.size;
								}
								else if (_field_item == item) {
									break;
								}
						}
						
						if (length < 0) return false;
						if (item < this.field_item_0 || this.field_items == 0) {
							this.BIT        = this.BIT_S - fld.field_info_bits;
							this.BYTE       = this.BYTE_S;
							const ins_items = this.field_items == 0 ? 1 : this.field_item_0 - item;
							this.insert(this.field_bit, ins_items + len_bits, length * fld.size);
							this.BIT = this.BIT_S - fld.field_info_bits - 1;
							set_bits((this.field_item_0 = item), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits + fld.sparse_bits);
							set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
						}
						else if (item < this.field_item_0 + this.field_items) {
							if (item < this.field_item) {
								this.field_item = this.field_item_0;
								this.BIT        = this.BIT_S - fld.field_info_bits - 1;
								this.BYTE       = this.BYTE_S;
							}
							else {
								this.BIT--;
								this.field_item++;
								this.BYTE += this.item_len * fld.size;
							}
							
							for (; this.field_item < item; this.BIT--, this.field_item++)
								if ((this.bytes[this.BIT >> 3] & (1 << (this.BIT & 7))) != 0) {
									this.BIT -= len_bits;
									this.item_len = get_bits(this.bytes, this.BIT, len_bits);
									this.BYTE += this.item_len * fld.size;
								}
							
							this.insert(this.field_bit, len_bits, length * fld.size);
						}
						else {
							const ins_items = item - (this.field_item_0 + this.field_items) + 1;
							this.BIT        = this.BIT_E;
							this.BYTE       = this.BYTE_E;
							this.insert(this.field_bit, ins_items + len_bits, length * fld.size);
							this.BIT  = this.BIT_E + len_bits;
							this.BYTE = this.BYTE_E - length * fld.size;
							set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
						}
						
						this.bytes[this.BIT >> 3] |= 1 << (this.BIT & 7);
						set_bits((this.item_len = length), len_bits, this.bytes, (this.BIT -= len_bits));
						break;
					case 8:
					case 10:
						if (this.field_item_0 <= item && item < this.field_item_0 + this.field_items) {
							if (item < _field_item) {
								_field_item = this.field_item_0;
								bit         = this.BIT_S - fld.field_info_bits - 1;
							}
							else {
								bit--;
								_field_item++;
							}
							
							for (const bits_ = this.item_len * fld.size; ; bit--, _field_item++)
								if ((this.bytes[bit >> 3] & (1 << (bit & 7))) != 0) {
									bit -= bits_;
									if (_field_item == item) {
										this.field_item = item;
										this.BIT        = bit;
										return true;
									}
								}
								else if (_field_item == item) {
									break;
								}
						}
						
						if (length < 0) return false;
						let bits = this.item_len * fld.size;
						if (item < this.field_item_0 || this.field_items == 0) {
							this.BIT        = this.BIT_S - fld.field_info_bits;
							this.BYTE       = this.BYTE_S;
							const ins_items = this.field_items == 0 ? 1 : this.field_item_0 - item;
							this.insert(this.field_bit, ins_items + bits, 0);
							this.BIT = this.BIT_S - fld.field_info_bits - 1;
							set_bits((this.field_item_0 = item), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits + fld.sparse_bits);
							set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
						}
						else if (item < this.field_item_0 + this.field_items) {
							if (item < this.field_item) {
								this.field_item = this.field_item_0;
								this.BIT        = this.BIT_S - fld.field_info_bits - 1;
							}
							else {
								this.BIT--;
								this.field_item++;
							}
							
							for (; this.field_item < item; this.BIT--, this.field_item++) if ((this.bytes[this.BIT >> 3] & (1 << (this.BIT & 7))) != 0) this.BIT -= bits;
							
							this.insert(this.field_bit, bits, 0);
						}
						else {
							const ins_items = item + 1 - (this.field_item_0 + this.field_items);
							this.BIT        = this.BIT_E;
							this.insert(this.field_bit, ins_items + bits, 0);
							this.BIT = this.BIT_E + bits;
							set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
						}
						
						this.bytes[this.BIT >> 3] |= 1 << (this.BIT & 7);
						this.BIT -= bits;
						break;
					case 11:
						if (item < this.field_items_total) {
							if (item < _field_item) {
								_field_item = -1;
								bit         = this.BIT_S - fld.field_info_bits;
							}
							
							let _items;
							do {
								bits = fld.size * (_items = get_bits(this.bytes, (bit -= len_bits), len_bits));
								bit -= bits;
								_field_item++;
							} while (_field_item < item);
							
							if (0 < _items) {
								this.BIT        = bit;
								this.item_len   = _items;
								this.field_item = _field_item;
								return true;
							}
						}
						
						if (length < 0) return false;
						if (this.field_items_total - 1 < item) throw new Error('No room for item=' + item + '. The field_items_total =' + this.field_items_total);
						if (item < this.field_item) {
							this.field_item = -1;
							this.BIT        = this.BIT_S - fld.field_info_bits;
						}
						
						while (this.field_item < item) {
							bits = get_bits(this.bytes, (this.BIT -= len_bits), len_bits) * fld.size;
							this.BIT -= bits;
							this.field_item++;
						}
						
						if (get_bits(this.bytes, this.BIT, len_bits) != 0) throw new Error('Already allocated');
						this.insert(this.field_bit, length * fld.size, 0);
						set_bits(length, len_bits, this.bytes, this.BIT);
						this.BIT -= length * fld.size;
						this.item_len = length;
						break;
					case 12:
						if (this.field_item_0 <= item && item < this.field_item_0 + this.field_items) {
							if (item < _field_item) {
								_field_item = this.field_item_0;
								bit         = this.BIT_S - fld.field_info_bits - 1;
							}
							else {
								bit--;
								_field_item++;
							}
							
							for (; ; bit--, _field_item++)
								if ((this.bytes[bit >> 3] & (1 << (bit & 7))) != 0) {
									bit -= len_bits;
									if (_field_item == item) {
										this.field_item = _field_item;
										this.item_len   = get_bits(this.bytes, bit, len_bits);
										this.BIT        = bit - this.item_len * fld.size;
										return true;
									}
									
									bit -= get_bits(this.bytes, bit, len_bits) * fld.size;
								}
								else if (_field_item == item) {
									break;
								}
						}
						
						if (length < 0) return false;
						if (item < this.field_item_0 || this.field_items == 0) {
							this.BIT        = this.BIT_S - fld.field_info_bits;
							const ins_items = this.field_items == 0 ? 1 : this.field_item_0 - item;
							this.insert(this.field_bit, ins_items + len_bits + length * fld.size, 0);
							this.BIT = this.BIT_S - fld.field_info_bits - 1;
							set_bits((this.field_item_0 = item), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits + fld.sparse_bits);
							set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
						}
						else if (item < this.field_item_0 + this.field_items) {
							if (item < this.field_item) {
								this.field_item = this.field_item_0;
								this.BIT        = this.BIT_S - fld.field_info_bits - 1;
							}
							else {
								this.BIT--;
								this.field_item++;
							}
							
							for (; this.field_item < item; this.BIT--, this.field_item++)
								if ((this.bytes[this.BIT >> 3] & (1 << (this.BIT & 7))) != 0) {
									this.BIT -= len_bits;
									this.item_len = get_bits(this.bytes, this.BIT, len_bits);
									this.BIT -= this.item_len * fld.size;
								}
							
							this.insert(this.field_bit, len_bits + length * fld.size, 0);
						}
						else {
							const ins_items = item - (this.field_item_0 + this.field_items) + 1;
							this.BIT        = this.BIT_E;
							this.insert(this.field_bit, ins_items + len_bits + length * fld.size, 0);
							this.BIT = this.BIT_E + length * fld.size + len_bits;
							set_bits((this.field_items += ins_items), fld.sparse_bits, this.bytes, this.BIT_S - fld.field_info_bits);
						}
						
						this.bytes[this.BIT >> 3] |= 1 << (this.BIT & 7);
						set_bits((this.item_len = length), len_bits, this.bytes, (this.BIT -= len_bits));
						this.BIT -= length * fld.size;
						break;
				}
				
				this.field_item = item;
				return false;
			}
			
			private resize_bytes(diff: number) {
				let cur: Cursor = this;
				while (cur.prev != null) cur = <Cursor>cur.prev;
				if (cur.pack_LAST_BYTE < 0) cur.set_pack_LASTS();
				const new_buffer = copy_bytes(this.bytes.buffer, 0, new ArrayBuffer(cur.pack_LAST_BYTE + diff), 0, this.BYTE);
				if (diff < 0) copy_bytes(this.bytes.buffer, this.BYTE - diff, new_buffer, this.BYTE, cur.pack_LAST_BYTE - (this.BYTE - diff));
				else copy_bytes(this.bytes.buffer, this.BYTE, new_buffer, this.BYTE + diff, cur.pack_LAST_BYTE - this.BYTE);
				
				const new_bytes = (this.bytes = new DataView(new_buffer));
				for (; ;) {
					if (-1 < cur.pack_LAST_BYTE) cur.pack_LAST_BYTE += diff;
					cur.BYTE_E += diff;
					cur.bytes = new_bytes;
					if (cur === this) return;
					if (-1 < cur.field_bit) {
						const fld = cur.getField();
						cur.item_len += diff / fld.size;
						if (fld.datatypes == null) {
							const len_bits = -cur.getField().length;
							set_bits(cur.item_len, len_bits, this.bytes, cur.BIT);
						}
					}
					
					cur = <Cursor>cur.next_;
				}
			}
			
			private insert(fbit: number, bits: number, bytes: number) {
				if (this.field_bit != fbit) {
					this.BIT      = this.BIT_S = this.BIT_E;
					this.BYTE     = this.BYTE_S = this.BYTE_E;
					this.item_len = 0;
				}
				
				if (this.pack_LAST_BYTE < 0) this.set_pack_LASTS();
				let old_bytes_len = this.pack_LAST_BYTE;
				let add_to_bits_bytes;
				let len_bits      = this.meta.BITS_lenINbytes_bits;
				if (this.BIT == this.pack_LAST_BIT && this.BYTE == old_bytes_len) {
					add_to_bits_bytes = len_bits == 0 ? 0 : bits2bytes(fbit + bits - (this.pack_LAST_BIT - this.origin * 8) + 1);
					if (0 < add_to_bits_bytes || 0 < bytes) {
						this.resize_bytes(add_to_bits_bytes + bytes);
						if (0 < add_to_bits_bytes) {
							copy_bits(this.bytes, this.BIT, (old_bytes_len << 3) - this.BIT, this.bytes, this.BIT + (add_to_bits_bytes << 3));
							set_0(this.bytes, this.BIT, add_to_bits_bytes << 3);
						}
					}
				}
				else {
					let add_bits_bits = len_bits == 0 ? 0 : bits - (this.pack_LAST_BIT - (this.origin * 8 + this.pack_LAST_field_bit + 1));
					add_to_bits_bytes = len_bits == 0 ? 0 : bits2bytes(add_bits_bits);
					if (0 < add_to_bits_bytes || 0 < bytes) {
						this.resize_bytes(add_to_bits_bytes + bytes);
						if (0 < bits) {
							copy_bits(this.bytes, this.BIT, (this.BYTE << 3) - this.BIT, this.bytes, this.BIT + (add_to_bits_bytes << 3));
							
							copy_bits(this.bytes, this.pack_LAST_BIT, this.BIT - this.pack_LAST_BIT, this.bytes, this.pack_LAST_BIT + (add_to_bits_bytes << 3) - bits);
							
							if (0 < (add_to_bits_bytes << 3) - bits) set_0(this.bytes, this.pack_LAST_BIT, (add_to_bits_bytes << 3) - bits);
							set_0(this.bytes, this.BIT + (add_to_bits_bytes << 3) - bits, bits);
						}
					}
					else {
						copy_bits(this.bytes, this.pack_LAST_BIT, this.BIT - this.pack_LAST_BIT, this.bytes, this.pack_LAST_BIT - bits);
						set_0(this.bytes, this.BIT - bits, bits);
					}
				}
				
				if (0 < len_bits && 0 < add_to_bits_bytes) {
					let old_value = get_bits(this.bytes, this.origin * 8 + this.meta.field_0_bit - len_bits, len_bits);
					set_bits(old_value + add_to_bits_bytes, len_bits, this.bytes, this.origin * 8 + this.meta.field_0_bit - len_bits);
				}
				
				let zazor_delta = (add_to_bits_bytes << 3) - bits;
				this.pack_LAST_BIT += zazor_delta;
				this.BIT += add_to_bits_bytes << 3;
				this.BYTE += add_to_bits_bytes;
				if (fbit == this.field_bit) {
					this.BIT_E += zazor_delta;
					this.BIT_S += add_to_bits_bytes << 3;
					this.BYTE_S += add_to_bits_bytes;
				}
				else {
					this.BIT_S  = this.BIT_E = this.BIT;
					this.BYTE_E = this.BYTE_S = this.BYTE;
					this.bytes[origin + (fbit >> 3)] |= 1 << (fbit & 7);
					if (this.pack_LAST_field_bit < fbit) this.pack_LAST_field_bit = fbit;
					this.field_bit = fbit;
				}
			}
		}
		
		export namespace Cursor {
			export class View {
				public data_: Cursor;
				
				constructor(data?: Cursor) {
					this.data_ = data!;
				}
			}
			
			const utf8decoder = new TextDecoder();
			const utf8encoder = new TextEncoder();
			
			export class UTF8 extends Uint8Array {
				constructor(cur: Cursor, src?: string, index: number = 0) {
					if (src) {
						let size = 0;
						
						for (let Len = src.length, ch = 0, ch2 = 0, i = 0; i !== Len;) {
							ch = src.charCodeAt(i);
							i += 1;
							if (0xd800 <= ch && ch <= 0xdbff) {
								if (i === Len) {
									size += 3;
									break;
								}
								ch2 = src.charCodeAt(i);
								if (0xdc00 <= ch2 && ch2 <= 0xdfff) {
									ch = (ch - 0xd800) * 0x400 + ch2 - 0xdc00 + 0x10000;
									i += 1;
									if (0xffff < ch) {
										size += 4;
										continue;
									}
								}
								else size += 3;
							}
							if (ch <= 0x007f) size += 1;
							else if (ch <= 0x07ff) size += 2;
							else size += 3;
						}
						
						if (index < 0) cur.set_field(-index - 1, size);
						else cur.set_item(index, size);
						
						super(cur.bytes.buffer, cur.BYTE, size);
						
						utf8encoder.encodeInto(src, this);
					}
					else super(cur.bytes.buffer, cur.BYTE, cur.item_len);
				}
				
				toString(): string {
					return utf8decoder.decode(this);
				}
			}
		}
	}
	
	export namespace Channel {
		export function bytes_transfer(src: Transmitter, dst: Receiver, buff: Uint8Array = new Uint8Array(new ArrayBuffer(1024))) {
			for (let len = 0, max = buff.byteLength; 0 < (len = src.PacksIntoBytes(buff, 0, max)); dst.BytesIntoPacks(buff, 0, len)) ;
		}
		
		const BR                         = 0x55;
		const tab: ReadonlyArray<number> = [0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548, 53677, 57806, 61935];
		
		function crc16(data: number, crc: number): number {
			crc = 0xffff & (tab[((crc >> 12) ^ (data >> 4)) & 0x0f] ^ (crc << 4));
			return 0xffff & (tab[((crc >> 12) ^ (data & 0x0f)) & 0x0f] ^ (crc << 4));
		}
		
		const tab32 = (function () {
			let c;
			const ret = new Uint32Array(new ArrayBuffer(255 * 4));
			for (let n = 0; n < 256; n++) {
				c = n;
				for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
				
				ret[n] = c;
			}
			return ret;
		})();
		
		export function crc32(data: number, crc?: number): number {
			let ret = crc || 0 ^ -1;
			
			ret = (ret >>> 8) ^ tab32[(ret ^ data) & 0xff];
			
			return (ret ^ -1) >>> 0;
		}
		
		export class Flow {
			readonly curs: Pack.Meta.Field.CursorBase;
			cur: Pack.Meta.Field.CursorBase;
			UvalueH = 0;
			Uvalue  = 0;
			crc     = 0;
			
			state: Flow.STATE = Flow.STATE.STANDBY;
			
			constructor(nested_max: number) {
				this.cur    = this.curs = new Pack.Meta.Field.CursorBase(nested_max);
				this.state  = Flow.STATE.STANDBY;
				this.mode   = Flow.MODE.NONE;
				this.Uvalue = 0;
				this.crc    = 0;
			}
			
			pullUInt(src: DataView, byte: number, bytes: number) {
				switch (bytes) {
					case 2:
						this.Uvalue  = src.getUint16(byte, true);
						this.UvalueH = 0;
						break;
					case 4:
						this.Uvalue  = src.getUint32(byte, true);
						this.UvalueH = 0;
						break;
					case 8:
						this.Uvalue  = src.getUint32(byte, true);
						this.UvalueH = src.getUint32(byte + 4, true);
				}
			}
			
			pushUInt(dst: DataView, byte: number, bytes: number) {
				switch (bytes) {
					case 2:
						dst.setUint16(byte, this.Uvalue, true);
						break;
					case 4:
						dst.setUint32(byte, this.Uvalue, true);
						break;
					case 8:
						dst.setUint32(byte, this.Uvalue, true);
						dst.setUint32(byte + 4, this.UvalueH, true);
				}
			}
			
			mode: Flow.MODE = Flow.MODE.NONE;
			
			public next(): Pack.Meta.Field.CursorBase | null {
				let fb = -1;
				for (let to_end = false; ;) {
					start: {
						if (this.cur.field_bit == -1) {
							switch (this.cur.item_type) {
								case 0:
									this.cur.BYTE_S = this.cur.BYTE_E = this.cur.origin;
									if (0 < this.cur.meta._2) {
										this.cur.BYTE_E += this.cur.meta._2 * (this.cur.item_type = 2);
										this.state = Flow.STATE.VARINT;
										return this.cur;
									}
								case 2:
									if (0 < this.cur.meta._4) {
										this.cur.BYTE_E += this.cur.meta._4 * (this.cur.item_type = 4);
										this.state = Flow.STATE.VARINT;
										return this.cur;
									}
								case 4:
									if (0 < this.cur.meta._8) {
										this.cur.BYTE_E += this.cur.meta._8 * (this.cur.item_type = 8);
										this.state = Flow.STATE.VARINT;
										return this.cur;
									}
								case 8:
									if (this.cur.BYTE_S < (this.cur.BYTE_E = this.cur.origin + this.cur.meta.packMinBytes)) {
										this.state         = Flow.STATE.BYTES;
										this.cur.item_type = 1;
										return this.cur;
									}
									this.mode = Flow.MODE.NONE;
								default:
									if (this.mode == Flow.MODE.NONE) {
										if (this.cur.meta.fields == null) {
											to_end = true;
											break;
										}
										let fix = this.cur.BYTE_S;
										this.cur.reset();
										if (fix < this.cur.BYTE_E) {
											this.cur.BYTE_S    = fix;
											this.state         = Flow.STATE.BYTES;
											this.mode          = Flow.MODE.OPTS_INFO;
											this.cur.item_type = 1;
											return this.cur;
										}
									}
									
									this.mode      = Flow.MODE.NONE;
									this.cur.BIT_E = this.cur.BIT_S = (this.cur.BYTE_S = this.cur.BYTE_E) << 3;
									if ((fb = this.cur.next_field_bit()) < 0) to_end = true;
							}
						}
						else if ((fb = this.cur.next_field_bit()) < 0) to_end = true;
						
						do {
							next_field: {
								if (!to_end)
									do {
										this.cur.field_bit = fb;
										let fld            = this.cur.getField();
										this.state         = fld.varint ? Flow.STATE.VARINT : Flow.STATE.BYTES;
										this.cur.BIT_S     = this.cur.BIT_E;
										this.cur.BYTE_S    = this.cur.BYTE_E;
										
										if (fld.datatypes == null) {
											if (0 < fld.length)
												if (0 < fld.size) this.cur.BYTE_E += fld.const_dims_total * fld.length * (this.cur.item_type = fld.size);
												else this.cur.BIT_E += fld.const_dims_total * fld.length * fld.size;
											else this.cur.set_E(fld);
											if (this.cur.BYTE_S < this.cur.BYTE_E) return this.cur;
										}
										else {
											let prev           = this.cur;
											this.cur           = this.cur.next_!;
											this.cur.origin    = prev.BYTE_E;
											this.cur.bytes     = prev.bytes;
											this.cur.field_bit = -1;
											this.cur.item_type = 0;
											
											prev.item_type = fld.const_dims_total;
											if (fld.var_dims != null) for (let i = 0, bit = prev.BIT_S; i < fld.var_dims.length; i++) prev.item_type *= get_bits(prev.bytes, (bit -= fld.var_dims[i]), fld.var_dims[i]);
											switch (fld.type) {
												case 1:
													this.cur.meta = fld.datatypes[0];
													break start;
												case 3:
													prev.BIT_E -= fld.field_info_bits;
													this.cur.meta = fld.datatypes[get_bits(prev.bytes, prev.BIT_E, -fld.length)];
													break start;
												case 5:
													prev.BIT_E -= fld.field_info_bits;
													for (let item_type, len_bits = -fld.length; 0 < prev.item_type; prev.item_type--)
														if (0 < (item_type = get_bits(prev.bytes, (prev.BIT_E -= len_bits), len_bits))) {
															this.cur.meta = fld.datatypes[item_type - 1];
															break start;
														}
											}
										}
									} while (-1 < (fb = this.cur.next_field_bit()));
								else to_end = false;
								
								for (let prev = this.cur.prev; prev != null;) {
									if (1 < prev.item_type) {
										let fld = prev.getField();
										for (let item_type, len_bits = -fld.length; 1 < prev.item_type--;) {
											if (fld.type == 5)
												if (0 < (item_type = get_bits(prev.bytes, (prev.BIT_E -= len_bits), len_bits))) this.cur.meta = fld.datatypes[item_type - 1];
												else continue;
											this.cur.origin    = this.cur.BYTE_E;
											this.cur.field_bit = -1;
											this.cur.item_type = 0;
											break start;
										}
									}
									
									prev.BYTE_E = this.cur.BYTE_E;
									this.cur    = prev;
									if (-1 < (fb = this.cur.next_field_bit())) break next_field;
									prev = this.cur.prev;
								}
								
								return null;
							}
						} while (true);
					}
				}
			}
		}
		
		export namespace Flow {
			export const enum STATE {
				STANDBY,
				PACK_ID,
				VARINT,
				VARINT_BR,
				BYTES,
				BYTES_BR
			}
			
			export enum MODE {
				OPTS_INFO,
				SET,
				NONE,
				CRC
			}
		}
		
		export abstract class Transmitter {
			readonly flow: Flow;
			readonly id_bytes: number;
			bits = 0;
			
			protected abstract pullSendingPack(): Pack | null;
			
			protected constructor(nested_max: number, id_bytes: number) {
				this.flow     = new Flow(nested_max);
				this.id_bytes = id_bytes;
			}
			
			public event: (reason: string) => void;
			
			public PacksIntoBytes(dst: Uint8Array, BYTE: number, bytes: number): number {
				const fix = BYTE;
				for (let cur = this.flow.cur; 0 < bytes--; BYTE++) {
					switch (this.flow.state) {
						case Flow.STATE.STANDBY:
							const pack = this.pullSendingPack();
							if (pack == null) return BYTE - fix;
							cur.wrap(pack);
							cur.field_bit    = 8 * (this.id_bytes - 1);
							this.flow.state  = Flow.STATE.PACK_ID;
							this.flow.Uvalue = pack.meta.id;
						
						case Flow.STATE.PACK_ID:
							dst[BYTE] = 0xff & (this.flow.Uvalue >> cur.field_bit);
							if (-1 < (cur.field_bit -= 8)) continue;
							cur.field_bit  = -1;
							this.flow.mode = Flow.MODE.NONE;
							if (cur.meta.fields != null) {
								this.flow.state  = Flow.STATE.VARINT;
								this.flow.Uvalue = cur.type_len(cur.meta, cur.origin) + 1 - (cur.origin + cur.meta.packMinBytes);
								cur.BIT_E        = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
								continue;
							}
							
							this.flow.Uvalue = 0;
							cur.BIT_E        = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
							break;
						case Flow.STATE.BYTES:
							dst[BYTE] = cur.bytes[cur.BYTE_S++];
							if (cur.BYTE_S < cur.BYTE_E) continue;
							break;
						case Flow.STATE.VARINT:
							if ((this.flow.Uvalue & ~0x7f) != 0) {
								dst[BYTE] = (this.flow.Uvalue & 0x7f) | 0x80;
								this.flow.Uvalue >>= 7;
								continue;
							}
							
							dst[BYTE] = this.flow.Uvalue;
							if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E) {
								this.flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_type);
								continue;
							}
							
							break;
					}
					
					if ((cur = this.flow.next()!) != null) {
						if (this.flow.state == Flow.STATE.VARINT) this.flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_type);
						continue;
					}
					
					this.flow.state = Flow.STATE.STANDBY;
					this.flow.mode  = Flow.MODE.NONE;
					cur             = this.flow.curs;
					do {
						cur.meta  = null!;
						cur.bytes = null!;
					} while ((cur = cur.next_!) != null);
					
					cur = this.flow.curs;
				}
				
				return BYTE - fix;
			}
		}
		
		export namespace Transmitter {
			export abstract class Advanced extends Transmitter {
				protected constructor(nested_max: number, id_bytes: number) {
					super(nested_max, id_bytes);
				}
				
				PacksIntoBytes(dst: Uint8Array, BYTE: number, bytes: number): number {
					const fix = BYTE;
					let cur   = this.flow.cur;
					for (let t; 0 < bytes--; t = this.flow.mode != Flow.MODE.CRC ? (this.flow.crc = crc16(dst[BYTE], this.flow.crc)) : 0, BYTE++) {
						switch (this.flow.state) {
							case Flow.STATE.STANDBY:
								const pack = this.pullSendingPack();
								if (pack == null) return BYTE - fix;
								cur.wrap(pack);
								cur.field_bit    = 8 * (this.id_bytes - 1);
								this.flow.state  = Flow.STATE.PACK_ID;
								this.flow.Uvalue = pack.meta.id;
								this.flow.crc    = 0;
								dst[BYTE]        = BR;
								continue;
							case Flow.STATE.PACK_ID:
								dst[BYTE] = 0xff & (this.flow.Uvalue >> cur.field_bit);
								if (-1 < (cur.field_bit -= 8)) continue;
								cur.field_bit  = -1;
								this.flow.mode = Flow.MODE.NONE;
								if (cur.meta.fields != null) {
									this.flow.state  = Flow.STATE.VARINT;
									this.flow.Uvalue = cur.type_len(cur.meta, cur.origin) + 1 - (cur.origin + cur.meta.packMinBytes);
									cur.BIT_E        = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
									continue;
								}
								
								this.flow.Uvalue = 0;
								cur.BIT_E        = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
								break;
							case Flow.STATE.BYTES:
								if (this.flow.mode == Flow.MODE.CRC) {
									switch (cur.item_type) {
										case 4:
											cur.item_type = (dst[BYTE] = this.flow.crc >> 8) == BR ? 3 : 2;
											continue;
										case 3:
											dst[BYTE]     = BR;
											cur.item_type = 2;
											continue;
										case 2:
											if ((dst[BYTE] = this.flow.crc & 0xff) != BR) break;
											cur.item_type = 1;
											continue;
										case 1:
											dst[BYTE] = BR;
											break;
									}
									
									this.flow.state = Flow.STATE.STANDBY;
									this.flow.mode  = Flow.MODE.NONE;
									cur             = this.flow.curs;
									do {
										cur.meta  = null!;
										cur.bytes = null!;
									} while ((cur = cur.next_!) != null);
									
									cur = this.flow.curs;
									continue;
								}
								
								if ((this.flow.Uvalue = dst[BYTE] = cur.bytes[cur.BYTE_S++]) == BR) {
									this.flow.state = Flow.STATE.BYTES_BR;
									continue;
								}
							
							case Flow.STATE.BYTES_BR:
								this.flow.state = Flow.STATE.BYTES;
								dst[BYTE]       = this.flow.Uvalue;
								if (cur.BYTE_S < cur.BYTE_E) continue;
								break;
							case Flow.STATE.VARINT:
								if ((this.flow.Uvalue & ~0x7f) != 0) {
									dst[BYTE] = (this.flow.Uvalue & 0x7f) | 0x80;
									this.flow.Uvalue >>= 7;
									continue;
								}
								
								if (this.flow.Uvalue == BR) {
									this.flow.state = Flow.STATE.VARINT_BR;
									dst[BYTE]       = BR;
									continue;
								}
							
							case Flow.STATE.VARINT_BR:
								this.flow.state = Flow.STATE.VARINT;
								dst[BYTE]       = this.flow.Uvalue;
								if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E) {
									this.flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_type);
									continue;
								}
								
								break;
						}
						
						if ((cur = this.flow.next()!) != null) {
							if (this.flow.state == Flow.STATE.VARINT) this.flow.Uvalue = get_bytes(cur.bytes, cur.BYTE_S, cur.item_type);
							continue;
						}
						
						cur             = this.flow.curs;
						this.flow.state = Flow.STATE.BYTES;
						this.flow.mode  = Flow.MODE.CRC;
						cur.item_type   = 4;
					}
					
					return BYTE - fix;
				}
			}
		}
		
		export abstract class Receiver {
			bits = 0;
			time = 0;
			
			readonly flow: Flow;
			readonly id_bytes: number;
			
			public event: (reason: string) => void;
			
			protected constructor(nested_max: number, id_bytes: number) {
				this.flow     = new Flow(nested_max);
				this.id_bytes = id_bytes;
			}
			
			protected abstract dispatch(id: number, pack?: Pack): Pack.Meta | null;
			
			public BytesIntoPacks(src: Uint8Array, BYTE: number, bytes: number) {
				this.time = ~0;
				if (this.time == 0) {
					this.time       = ~0;
					this.flow.state = Flow.STATE.STANDBY;
					this.event?.call('Receive timeout');
				}
				
				for (let cur = this.flow.cur; 0 < bytes--; BYTE++) {
					switch (this.flow.state) {
						case Flow.STATE.STANDBY:
							this.flow.Uvalue = 0;
							this.bits        = 0;
							this.flow.state  = Flow.STATE.PACK_ID;
						case Flow.STATE.PACK_ID:
							this.flow.Uvalue = (this.flow.Uvalue << 8) | src[BYTE];
							if (++this.bits < this.id_bytes) continue;
							this.flow.mode = Flow.MODE.NONE;
							const meta     = this.dispatch(this.flow.Uvalue);
							if (meta == null) {
								this.event?.call('Unrecognized package ID = ' + this.flow.Uvalue);
								this.flow.state = Flow.STATE.STANDBY;
								continue;
							}
							
							cur.meta         = meta;
							this.flow.Uvalue = 0;
							this.bits        = 0;
							cur.field_bit    = -1;
							cur.BIT_E        = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
							if (meta.fields != null) {
								this.flow.state = Flow.STATE.VARINT;
								this.flow.mode  = Flow.MODE.OPTS_INFO;
								continue;
							}
							else {
								cur.bytes = new DataView(new ArrayBuffer(meta.packMinBytes));
								cur.reset();
							}
							
							break;
						case Flow.STATE.BYTES:
							cur.bytes[cur.BYTE_S++] = src[BYTE];
							if (cur.BYTE_S < cur.BYTE_E) continue;
							break;
						case Flow.STATE.VARINT:
							this.flow.Uvalue |= (src[BYTE] & 0x7f) << this.bits;
							this.bits += 7;
							if ((src[BYTE] & 0x80) != 0) continue;
							this.bits = 0;
							if (this.flow.mode == Flow.MODE.OPTS_INFO) {
								cur.bytes        = new DataView(new ArrayBuffer(cur.meta.packMinBytes + this.flow.Uvalue));
								this.flow.Uvalue = 0;
								this.flow.mode   = Flow.MODE.NONE;
								break;
							}
							
							set_bytes(this.flow.Uvalue, cur.item_type, cur.bytes, cur.BYTE_S);
							this.flow.Uvalue = 0;
							if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E) continue;
							break;
					}
					
					if ((cur = this.flow.next()!) != null) continue;
					cur = this.flow.curs;
					this.dispatch(cur.meta.id, cur.unwrap());
					cur = this.flow.curs;
					do {
						cur.meta  = null!;
						cur.bytes = null!;
					} while ((cur = cur.next_!) != null);
					
					cur             = this.flow.curs;
					this.flow.state = Flow.STATE.STANDBY;
					this.flow.mode  = Flow.MODE.NONE;
				}
			}
		}
		
		export namespace Receiver {
			export abstract class Advanced extends Receiver {
				protected constructor(nested_max: number, id_bytes: number) {
					super(nested_max, id_bytes);
				}
				
				bits = 0;
				
				public BytesIntoPacks(src: Uint8Array, BYTE: number, bytes: number) {
					this.time = ~0;
					if (this.time == 0) {
						this.time       = ~0;
						this.flow.state = Flow.STATE.STANDBY;
						this.event?.call('Receive timeout');
					}
					
					let cur = this.flow.cur;
					for (let t; 0 < bytes--; t = this.flow.mode != Flow.MODE.CRC ? (this.flow.crc = crc16(src[BYTE], this.flow.crc)) : 0, BYTE++) {
						switch (this.flow.state) {
							case Flow.STATE.STANDBY:
								this.flow.crc    = 0;
								this.flow.Uvalue = 0;
								this.bits        = 0;
								if (src[BYTE] == BR) this.flow.state = Flow.STATE.PACK_ID;
								continue;
							case Flow.STATE.PACK_ID:
								if (src[BYTE] == BR) {
									this.event?.call(' After BR expect helper ID but got +BR');
									this.flow.state = Flow.STATE.STANDBY;
									continue;
								}
								
								this.flow.Uvalue = (this.flow.Uvalue << 8) | src[BYTE];
								if (++this.bits < this.id_bytes) continue;
								this.flow.mode = Flow.MODE.NONE;
								let meta       = this.dispatch(this.flow.Uvalue);
								if (meta == null) {
									this.event?.call('Unrecognized package ID = ' + this.flow.Uvalue);
									this.flow.state = Flow.STATE.STANDBY;
									continue;
								}
								
								cur.meta         = meta;
								this.flow.Uvalue = 0;
								this.bits        = 0;
								cur.field_bit    = -1;
								cur.BIT_E        = cur.BIT_S = cur.BYTE_E = cur.BYTE_S = cur.item_type = 0;
								if (meta.fields != null) {
									this.flow.state = Flow.STATE.VARINT;
									this.flow.mode  = Flow.MODE.OPTS_INFO;
									continue;
								}
								else {
									cur.bytes = new DataView(new ArrayBuffer(meta.packMinBytes));
									cur.reset();
								}
								
								break;
							case Flow.STATE.BYTES:
								if (src[BYTE] == BR) {
									this.flow.state = Flow.STATE.BYTES_BR;
									continue;
								}
							
							case Flow.STATE.BYTES_BR:
								if (this.flow.state == Flow.STATE.BYTES_BR) {
									if (src[BYTE] != BR) {
										this.event?.call('waiting for second BR but got ' + src[BYTE]);
										this.flow.state = Flow.STATE.STANDBY;
										continue;
									}
									
									this.flow.state = Flow.STATE.BYTES;
								}
								
								if (this.flow.mode == Flow.MODE.CRC)
									switch (cur.item_type) {
										case 2:
											this.flow.Uvalue = src[BYTE] << 8;
											cur.item_type    = 1;
											continue;
										case 1:
											if ((this.flow.Uvalue | src[BYTE]) == this.flow.crc) this.dispatch(cur.meta.id, cur.unwrap());
											else this.event?.call('CRC error');
											cur = this.flow.curs;
											do {
												cur.meta  = null!;
												cur.bytes = null!;
											} while ((cur = cur.next_!) != null);
											
											this.flow.state = Flow.STATE.STANDBY;
											this.flow.mode  = Flow.MODE.NONE;
											continue;
									}
								
								cur.bytes[cur.BYTE_S++] = src[BYTE];
								if (cur.BYTE_S < cur.BYTE_E) continue;
								break;
							case Flow.STATE.VARINT:
								if (src[BYTE] == BR) {
									this.flow.state = Flow.STATE.VARINT_BR;
									continue;
								}
							
							case Flow.STATE.VARINT_BR:
								if (this.flow.state == Flow.STATE.VARINT_BR) {
									if (src[BYTE] != BR) {
										this.event?.call('waiting for second BR but got ' + src[BYTE]);
										this.flow.state = Flow.STATE.STANDBY;
										continue;
									}
									
									this.flow.state = Flow.STATE.VARINT;
								}
								
								this.flow.Uvalue |= (src[BYTE] & 0x7f) << this.bits;
								this.bits += 7;
								if ((src[BYTE] & 0x80) != 0) continue;
								this.bits = 0;
								if (this.flow.mode == Flow.MODE.OPTS_INFO) {
									cur.bytes        = new DataView(new ArrayBuffer(cur.meta.packMinBytes + this.flow.Uvalue));
									this.flow.Uvalue = 0;
									this.flow.mode   = Flow.MODE.NONE;
									break;
								}
								
								set_bytes(this.flow.Uvalue, cur.item_type, cur.bytes, cur.BYTE_S);
								this.flow.Uvalue = 0;
								if ((cur.BYTE_S += cur.item_type) < cur.BYTE_E) continue;
								break;
						}
						
						if ((cur = this.flow.next()!) != null) continue;
						cur             = this.flow.curs;
						this.flow.state = Flow.STATE.BYTES;
						this.flow.mode  = Flow.MODE.CRC;
						cur.item_type   = 2;
					}
				}
			}
		}
	}
}
