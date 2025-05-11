<!-- 
=============================================
            MARKDOWN REFERENCE GUIDE
=============================================
This is a hidden comment (won't render in output).
Comments are useful for notes or metadata.
-->

# **Markdown Cheat Sheet**  
*A single-file reference for all Markdown syntax.*  

---

## **1. Headers**  
```markdown
# H1  
## H2  
### H3  
#### H4  
##### H5  
###### H6  
```

---

## **2. Text Formatting**  
```markdown
*Italic* or _Italic_  
**Bold** or __Bold__  
***Bold & Italic*** or ___Bold & Italic___  
~~Strikethrough~~  
`Inline code`  
```

---

## **3. Lists**  
### **Unordered**  
```markdown
- Item 1  
- Item 2  
  - Subitem (indent with 2 spaces)  
* Alternative bullet  
```

### **Ordered**  
```markdown
1. First item  
2. Second item  
   1. Subitem (indent with 3 spaces)  
```

---

## **4. Links & Images**  
```markdown
[Google](https://google.com)  
![Alt Text](image.jpg "Optional hover text")  
```

---

## **5. Code**  
### Inline  
`` `code` `` → `code`  

### Block  
````markdown
```python
print("Syntax highlighting!")
```
````


## **6. Tables**  

| Column 1 | Column 2 |  
|----------|----------|  
| Row 1    | Data     |  
| Row 2    | More     |  


## **7. Blockquotes**  
> Quote text here.  
> > Nested quote.  

## **8. Horizontal Rules** 
---  
***  
___  

## **9. Task Lists (GitHub Flavored)**  
- [x] Completed  
- [ ] Pending  

## **10. Footnotes**  
Here’s a footnote[^1].  

[^1]: Footnote text



## **11. Escaping**  
\*Escape special characters\* → *Escape special characters*  


## **12. HTML (Fallback)**
<details>
<summary>Click to expand</summary>
Hidden content!  
</details>
