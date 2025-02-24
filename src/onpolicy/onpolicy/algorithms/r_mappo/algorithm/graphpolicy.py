import math
import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.nn.utils.prune as prune

##############################################
# 1. 多头注意力模块及相关辅助
##############################################
class Normalization(nn.Module):
    def __init__(self, embedding_dim, eps=1e-6):
        super(Normalization, self).__init__()
        self.norm = nn.LayerNorm(embedding_dim, eps=eps)
    
    def forward(self, x):
        return self.norm(x)
    
class MultiHeadAttention(nn.Module):
    def __init__(self, embedding_dim, n_head):
        super(MultiHeadAttention, self).__init__()
        self.multihead_attn = nn.MultiheadAttention(embed_dim=embedding_dim, num_heads=n_head, batch_first=True)
        
    def forward(self, q, k=None, v=None, key_padding_mask=None, attn_mask=None):
        if k is None:
            k = q
        if v is None:
            v = q
        out, attn_weights = self.multihead_attn(q, k, v,
                                                key_padding_mask=key_padding_mask,
                                                attn_mask=attn_mask)
        return out, attn_weights

##############################################
# 2. Transformer Encoder / Decoder
##############################################
class EncoderLayer(nn.Module):
    def __init__(self, embedding_dim, n_head):
        super(EncoderLayer, self).__init__()
        self.multiHeadAttention = MultiHeadAttention(embedding_dim, n_head)
        self.normalization1 = Normalization(embedding_dim)
        self.feedForward = nn.Sequential(
            nn.Linear(embedding_dim, 512),
            nn.ReLU(inplace=True),
            nn.Linear(512, embedding_dim)
        )
        self.normalization2 = Normalization(embedding_dim)

    def forward(self, src, key_padding_mask=None, attn_mask=None):
        # src: (batch, seq_len, embedding_dim)
        h0 = src
        h = self.normalization1(src)
        h_attn, _ = self.multiHeadAttention(q=h, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        h = h0 + h_attn
        h1 = h
        h = self.normalization2(h)
        h_ff = self.feedForward(h)
        h2 = h1 + h_ff
        return h2

class DecoderLayer(nn.Module):
    def __init__(self, embedding_dim, n_head):
        super(DecoderLayer, self).__init__()
        self.multiHeadAttention = MultiHeadAttention(embedding_dim, n_head)
        self.normalization1 = Normalization(embedding_dim)
        self.feedForward = nn.Sequential(
            nn.Linear(embedding_dim, 512),
            nn.ReLU(inplace=True),
            nn.Linear(512, embedding_dim)
        )
        self.normalization2 = Normalization(embedding_dim)

    def forward(self, tgt, memory, key_padding_mask=None, attn_mask=None):
        # tgt: (batch, tgt_len=1, embedding_dim)
        # memory: (batch, seq_len, embedding_dim)
        h0 = tgt
        tgt_norm = self.normalization1(tgt)
        memory_norm = self.normalization1(memory)
        h_attn, attn_weights = self.multiHeadAttention(q=tgt_norm, k=memory_norm, v=memory_norm,
                                                       key_padding_mask=key_padding_mask,
                                                       attn_mask=attn_mask)
        h = h0 + h_attn
        h1 = h
        h = self.normalization2(h)
        h_ff = self.feedForward(h)
        h2 = h1 + h_ff
        return h2, attn_weights

class Encoder(nn.Module):
    def __init__(self, embedding_dim=128, n_head=8, n_layer=6):
        super(Encoder, self).__init__()
        self.layers = nn.ModuleList([EncoderLayer(embedding_dim, n_head) for _ in range(n_layer)])

    def forward(self, src, key_padding_mask=None, attn_mask=None):
        # src: (batch, seq_len, embedding_dim)
        for layer in self.layers:
            src = layer(src, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        return src

class Decoder(nn.Module):
    def __init__(self, embedding_dim=128, n_head=8, n_layer=1):
        super(Decoder, self).__init__()
        self.layers = nn.ModuleList([DecoderLayer(embedding_dim, n_head) for _ in range(n_layer)])

    def forward(self, tgt, memory, key_padding_mask=None, attn_mask=None):
        for layer in self.layers:
            tgt, attn_weights = layer(tgt, memory, key_padding_mask=key_padding_mask, attn_mask=attn_mask)
        return tgt, attn_weights

##############################################
# 3. PolicyNet (Graph Module) 示例
##############################################
class PolicyNet(nn.Module):
    """
    简化示例：假设用于图节点特征处理
    node_dim: 每个节点的特征维度
    embedding_dim: Transformer 的隐藏维度
    """
    def __init__(self, node_dim, embedding_dim):
        super(PolicyNet, self).__init__()
        self.initial_embedding = nn.Linear(node_dim, embedding_dim)
        self.encoder = Encoder(embedding_dim=embedding_dim, n_head=8, n_layer=6)
        self.decoder = Decoder(embedding_dim=embedding_dim, n_head=8, n_layer=1)
        # 将当前节点特征与解码器输出拼接后再映射
        self.current_embedding = nn.Linear(embedding_dim * 2, embedding_dim)
        # 最终我们输出一个 embedding 供后续拼接
        self.out_fc = nn.Linear(embedding_dim, embedding_dim)

    def encode_graph(self, node_inputs, node_padding_mask=None, edge_mask=None):
        """
        node_inputs: (batch, num_nodes, node_dim)
        返回: (batch, num_nodes, embedding_dim)
        """
        node_feature = self.initial_embedding(node_inputs)
        enhanced_node_feature = self.encoder(src=node_feature,
                                             key_padding_mask=node_padding_mask,
                                             attn_mask=edge_mask)
        return enhanced_node_feature

    def decode_state(self, enhanced_node_feature, current_index, node_padding_mask=None):
        """
        current_index: (batch, 1)，表示当前节点索引
        返回: current_node_feature, enhanced_current_node_feature
        形状分别为 (batch, 1, embedding_dim)
        """
        batch_size, num_nodes, embedding_dim = enhanced_node_feature.size()
        # 提取当前节点特征
        current_index_expanded = current_index.unsqueeze(-1).expand(batch_size, 1, embedding_dim)
        current_node_feature = torch.gather(enhanced_node_feature, 1, current_index_expanded)
        # 解码器：输入当前节点特征和全局节点特征
        enhanced_current_node_feature, _ = self.decoder(tgt=current_node_feature,
                                                        memory=enhanced_node_feature,
                                                        key_padding_mask=node_padding_mask)
        return current_node_feature, enhanced_current_node_feature

    def forward(self, node_inputs, node_padding_mask, edge_mask, current_index, **kwargs):
        """
        简化 forward，不考虑边掩码(current_edge)等。只输出融合后的 embedding。
        """
        enhanced_node_feature = self.encode_graph(node_inputs, node_padding_mask, edge_mask)
        current_node_feature, enhanced_current_node_feature = self.decode_state(
            enhanced_node_feature, current_index, node_padding_mask
        )
        # 将当前节点特征与解码结果拼接后再映射
        fused = self.current_embedding(torch.cat([current_node_feature, enhanced_current_node_feature], dim=-1))
        # 返回一个 embedding 供后续拼接
        out = self.out_fc(fused.squeeze(1))  # shape: (batch, embedding_dim)
        return out
