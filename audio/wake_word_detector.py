import re
import time
import threading
from collections import deque
from difflib import SequenceMatcher
import jieba
from pypinyin import pinyin, Style
import numpy as np

class WakeWordDetector:
    def __init__(self, target_wake_word="小安", confidence_threshold=0.6, verbose=False):
        """
        :param verbose: 是否输出初始化和检测日志
        """
        self.target_wake_word = target_wake_word
        self.confidence_threshold = confidence_threshold
        self.verbose = verbose
        
        # 生成变体
        self.wake_word_variants = self._generate_variants(target_wake_word)
        
        # 提取拼音
        self.target_pinyin = self._text_to_pinyin(target_wake_word)
        
        # 冷却机制
        self.last_wake_time = 0
        self.cooldown_period = 3.0
        self.detection_history = deque(maxlen=10)
        
        # ⚡ 关键修复：预热 jieba 分词，避免第一次唤醒时卡顿
        if self.verbose:
            print("⏳ 正在预热分词模型...")
        list(jieba.cut("预热分词"))
        
        # 只在 verbose=True 时输出
        if self.verbose:
            print("🎯 唤醒词检测器初始化完成")
            print(f"   目标唤醒词: {target_wake_word}")
            print(f"   变体数量: {len(self.wake_word_variants)}")
            print(f"   目标拼音: {self.target_pinyin}")
        
    def _generate_variants(self, wake_word):
        """生成唤醒词的多种变体"""
        variants = set()
        
        # 1. 原始词
        variants.add(wake_word)
        
        # 2. 拼音相似
        pinyin_variants = self._text_to_pinyin_variants(wake_word)
        variants.update(pinyin_variants)
        
        # 3. 常见错读 - 根据唤醒词动态生成
        common_misreads = ["小安", "小按", "小案", "小暗", "小岸", "小鞍", "小俺", "小氨", "小庵", "小谙", "小铵", "晓安", "笑安", "小啊"]
        
        variants.update(common_misreads)
        
        # 4. 拆分组合
        for i in range(1, len(wake_word)):
            part1 = wake_word[:i]
            part2 = wake_word[i:]
            if part1 and part2:
                variants.add(part1 + part2)
                variants.add(part2 + part1)
        
        return list(variants)
    
    def _text_to_pinyin_variants(self, text):
        """获取文本的拼音多种可能形式"""
        pinyin_list = pinyin(text, style=Style.NORMAL, heteronym=False)
        return [''.join(item[0] for item in pinyin_list)]
    
    def _text_to_pinyin(self, text):
        """获取文本的拼音字符串"""
        try:
            py_list = pinyin(text, style=Style.NORMAL, heteronym=False)
            return ' '.join([item[0] for item in py_list])
        except:
            return text
    
    def _calculate_text_similarity(self, text1, text2):
        """计算两个文本的相似度"""
        # 1. 直接字符串匹配
        if text1 == text2:
            return 1.0
            
        # 2. 编辑距离相似度
        edit_similarity = SequenceMatcher(None, text1, text2).ratio()
        
        # 3. 拼音相似度（修复：使用 _text_to_pinyin 方法）
        pinyin1 = self._text_to_pinyin(text1)
        pinyin2 = self._text_to_pinyin(text2)
        pinyin_similarity = SequenceMatcher(None, pinyin1, pinyin2).ratio()
        
        # 4. 包含关系检查
        contains_score = 0.0
        if text2 in text1 or text1 in text2:
            contains_score = 0.3
            
        #综合评分 (权重可调)
        final_score = (
            edit_similarity * 0.4 + 
            pinyin_similarity * 0.4 + 
            contains_score * 0.2
        )
        
        return final_score
    
    def _extract_wake_candidates(self, text):
        """从文本中提取可能的唤醒词候选"""
        if not text or len(text.strip()) == 0:
            return []
            
        candidates = []
        text = text.strip()
        
        if self.verbose:
            print(f"🔍 提取候选词，输入文本: '{text}'")
        
        # 1. 直接检查变体是否包含在文本中
        for variant in self.wake_word_variants:
            if variant in text:
                candidates.append(variant)
                if self.verbose:
                    print(f"   ✅ 直接匹配变体: '{variant}'")
        
        # 2. 滑动窗口提取所有两字组合
        if len(text) >= 2:
            for i in range(len(text) - 1):
                two_char = text[i:i+2]
                # 确保是两个有效字符
                if len(two_char) == 2 and not re.search(r'[a-zA-Z0-9\s\.,!?，。！？]', two_char):
                    candidates.append(two_char)
                    if self.verbose:
                        print(f"   🔸 滑动窗口提取: '{two_char}'")
        
        # 3. 分词后检查相邻词组合
        try:
            words = list(jieba.cut(text, cut_all=False))
            if self.verbose:
                print(f"   📝 分词结果: {words}")
            
            # 检查单个词
            for word in words:
                if len(word) == 2 and not re.search(r'[a-zA-Z0-9\s\.,!?，。！？]', word):
                    candidates.append(word)
                    if self.verbose:
                        print(f"   🔹 分词单词: '{word}'")
            
            # 检查相邻词组合
            for i in range(len(words) - 1):
                combined = words[i] + words[i + 1]
                if len(combined) == 2 and not re.search(r'[a-zA-Z0-9\s\.,!?，。！？]', combined):
                    candidates.append(combined)
                    if self.verbose:
                        print(f"   🔹 分词组合: '{combined}'")
                    
        except Exception as e:
            if self.verbose:
                print(f"   ❌ 分词错误: {e}")
        
        # 4. 去除标点和空格后再次提取
        clean_text = re.sub(r'[a-zA-Z0-9\s\.,!?，。！？]', '', text)
        if clean_text and len(clean_text) >= 2:
            for i in range(len(clean_text) - 1):
                two_char = clean_text[i:i+2]
                if len(two_char) == 2:
                    candidates.append(two_char)
                    if self.verbose:
                        print(f"   🧹 清理后提取: '{two_char}'")
        
        # 去重并返回
        unique_candidates = list(set(candidates))
        if self.verbose:
            print(f"   🎯 最终候选词: {unique_candidates}")
        
        return unique_candidates
    
    def detect_wake_word(self, recognized_text):
        """检测唤醒词"""
        current_time = time.time()
        
        if self.verbose:
            print(f"\n🔍 开始检测唤醒词: '{recognized_text}'")
        
        # 检查冷却时间
        if current_time - self.last_wake_time < self.cooldown_period:
            if self.verbose:
                remaining_cooldown = self.cooldown_period - (current_time - self.last_wake_time)
                print(f"⏳ 冷却时间未结束，剩余: {remaining_cooldown:.1f}秒")
            return False, 0.0, ""
            
        if not recognized_text or len(recognized_text.strip()) == 0:
            if self.verbose:
                print("❌ 输入文本为空")
            return False, 0.0, ""
            
        # 清理文本
        text = recognized_text.strip().replace(" ", "").replace("，", "").replace("。", "")
        if self.verbose:
            print(f"📝 清理后文本: '{text}'")
        
        # 添加到文本缓冲区
        self.detection_history.append((current_time, text))
        
        # 检查当前文本
        candidates = self._extract_wake_candidates(text)
        
        best_score = 0.0
        best_candidate = ""
        
        if self.verbose:
            print(f"🎯 开始相似度计算...")
        for candidate in candidates:
            score = self._calculate_text_similarity(candidate, self.target_wake_word)
            if self.verbose:
                print(f"   候选词: '{candidate}' → 相似度: {score:.3f}")
            if score > best_score:
                best_score = score
                best_candidate = candidate
        
        # 改进的历史组合检测逻辑
        if best_score < self.confidence_threshold:
            if self.verbose:
                print(f"🔄 当前最高分 {best_score:.3f} < 阈值 {self.confidence_threshold}，尝试近期组合...")
            
            # 获取最近的文本，但限制时间窗口和组合方式
            recent_entries = list(self.detection_history)[-3:]  # 最近3条
            
            # 只有在时间间隔合理的情况下才进行组合（比如5秒内）
            valid_entries = []
            for timestamp, old_text in recent_entries:
                if current_time - timestamp <= 5.0:  # 5秒时间窗口
                    valid_entries.append((timestamp, old_text))
            
            if len(valid_entries) >= 2:  # 至少需要2条记录才进行组合
                # 尝试不同的组合方式
                combined_candidates = []
                
                # 1. 只组合相邻的短文本（每个文本长度<=4个字符）
                short_texts = [text for _, text in valid_entries if len(text) <= 4]
                if len(short_texts) >= 2:
                    combined_short = "".join(short_texts[-2:])  # 最近的两个短文本
                    if self.verbose:
                        print(f"🔗 短文本组合: '{combined_short}'")
                    combined_candidates.extend(self._extract_wake_candidates(combined_short))
                
                # 2. 检查是否存在单字符匹配（如"小"+"安"的分割情况）
                for i in range(len(valid_entries) - 1):
                    text1 = valid_entries[i][1]
                    text2 = valid_entries[i + 1][1]
                    
                    # 只有当两个文本都很短时才组合
                    if len(text1) <= 3 and len(text2) <= 3:
                        mini_combined = text1 + text2
                        if self.verbose:
                            print(f"🔗 短句组合: '{text1}' + '{text2}' = '{mini_combined}'")
                        combined_candidates.extend(self._extract_wake_candidates(mini_combined))
                
                # 检查组合候选词
                if combined_candidates:
                    if self.verbose:
                        print(f"📝 历史组合候选词: {list(set(combined_candidates))}")
                    for candidate in set(combined_candidates):  # 去重
                        score = self._calculate_text_similarity(candidate, self.target_wake_word)
                        if self.verbose:
                            print(f"   历史候选词: '{candidate}' → 相似度: {score:.3f}")
                        if score > best_score:
                            best_score = score
                            best_candidate = candidate
                else:
                    if self.verbose:
                        print("📝 未找到有效的历史组合候选词")
            else:
                if self.verbose:
                    print("📝 历史记录不足或时间窗口超出，跳过组合检测")
        
        # 判断是否唤醒
        if best_score >= self.confidence_threshold:
            self.last_wake_time = current_time
            if self.verbose:
                print(f"✅ 唤醒成功！最佳候选: '{best_candidate}', 得分: {best_score:.3f}")
            
            # 唤醒成功后清空历史缓冲区，避免影响后续检测
            self.detection_history.clear()
            
            return True, best_score, best_candidate
        else:
            if self.verbose:
                print(f"❌ 未达到唤醒阈值，最佳得分: {best_score:.3f}")
            
        return False, best_score, best_candidate
    
    def reset_cooldown(self):
        """重置冷却时间（用于测试或手动重置）"""
        self.last_wake_time = 0
        # 测试时也清空缓冲区
        self.detection_history.clear()
        if self.verbose:
            print("🔄 冷却时间和历史缓冲区已重置")
    
    def get_debug_info(self):
        """获取调试信息"""
        return {
            "buffer_size": len(self.detection_history),
            "last_wake_time": self.last_wake_time,
            "target_pinyin": self.target_pinyin,
            "recent_texts": [item[1] for item in list(self.detection_history)[-5:]],
            "wake_variants": self.wake_word_variants[:10]  # 只显示前10个变体
        }

# 测试代码
def test_wake_word_detector():
    """测试唤醒词检测器"""
    detector = WakeWordDetector("小安", confidence_threshold=0.6, verbose=True)  # 测试时开启详细日志
    
    test_cases = [
        "小安你好",
        "小按，帮我一下", 
        "我想叫小案过来",
        "小暗边有什么",
        "hello小安world",
        "小朋友你好",
        "今天天气不错",
        "小 安",
        "小安小安",
        "小岸有什么用",
        "晓安快来"
    ]
    
    print("🧪 测试唤醒词检测器")
    print(f"目标唤醒词: 小安")
    print(f"置信度阈值: {detector.confidence_threshold}")
    print("=" * 60)
    
    for i, test_text in enumerate(test_cases, 1):
        print(f"\n📋 测试 {i}/{len(test_cases)} - 独立测试")
        
        # 每次测试前完全重置状态
        detector.reset_cooldown()
        
        is_wake, confidence, candidate = detector.detect_wake_word(test_text)
        
        status = "✅ 唤醒" if is_wake else "❌ 未唤醒"
        print(f"🏆 结果: {status} | 置信度: {confidence:.3f} | 候选词: '{candidate}' | 原文: '{test_text}'")
        
        print("-" * 40)
    
    # 显示最终调试信息
    print("\n📊 最终调试信息:")
    debug_info = detector.get_debug_info()
    for key, value in debug_info.items():
        print(f"  {key}: {value}")

if __name__ == "__main__":
    test_wake_word_detector()