def test(mask, x, y):
    index = y * mask.width + x
    byte_ind = index // 8
    bit_ind = 7 - (index % 8) # bitorder 'big'
    return bool(mask.mask[byte_ind] & (1 << bit_ind))
